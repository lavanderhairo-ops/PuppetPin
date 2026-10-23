# Puppet Pin for DaVinci Resolve

After Effects style puppet pins for Resolve. A triangle mesh is built from the image's
alpha, and pins bend it with an as-rigid-as-possible solver, so limbs bend instead of
smearing. Separate parts of the image get separate meshes.

Free and open source (MIT license, see [LICENSE](LICENSE)). Windows, DaVinci Resolve /
Resolve Studio, Edit and Fusion pages.

**Just want to use it?** Download `PuppetPin-1.8-Windows.zip` from
[Releases](https://github.com/lavanderhairo-ops/PuppetPin/releases), unzip it, close Resolve and run
`Install.bat`. The zip's `README.txt` is the user guide. The rest of this page is for
building and changing the code.

**Credits.** Written from scratch with Claude (Anthropic's AI). It uses the OpenFX SDK, see
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

Everything is one OpenFX plugin, `PuppetPin.ofx` (folder `src/ofx/`, engine in `src/core/`). It
works on the Edit page and as a node on the Fusion page; on the Fusion page its pins are regular
Fusion point controls (paths, trackers, expressions) and the click/drag overlay works too.

## Edit page (OFX)

1. Edit page: Effects > Open FX > Filters > **Puppet Pin**, drop it on a clip with alpha
   (a PNG cutout, for example). Opaque footage works too: the whole frame becomes the mesh.
2. In the viewer's bottom-left overlay menu choose **Open FX Overlay**.
3. Double-click on the image to add pins (up to 48), then drag pins to pose. With **Auto
   Keyframe** on, every drag sets a keyframe at the playhead.
4. Click an existing pin to select it (its settings are in the Inspector under *Pin N*).

Pin settings:

| Setting | What it does |
| --- | --- |
| Position / Rest Position | Where the pin is now / which point of the original image it holds (0..1 of the frame) |
| Radius | Pixels around the pin that move rigidly with it; needed for Rotation and Scale |
| Influence | How far the pin pulls, in pixels along the shape; beyond it the image stays as it would be without this pin (0 = no limit). Drag the green handle on its circle in the viewer |
| Influence Falloff | How soft the edge of the influence is (0 = hard edge) |
| Rotation, Scale | Rotate / scale the locked area |
| Strength | Lower values pull softer |
| Layer | Where parts overlap, the part around the higher-layer pin is drawn on top (equal layers: parts further out from the body's middle go on top) |

While a pin is dragged, the overlay draws a live preview of the pose (the image as flat-coloured
triangles): Resolve only re-renders a plugin after the mouse is released, and OFX points get no native
handles in the Fusion page, so this is the only live feedback a plugin can give.

**Selecting several pins**: drag a box from an empty spot, Shift/Ctrl+click pins (modifier state comes from
key events), or type them into *Selected Pins* (string param; it only counts while it lists *Selected Pin*,
the pivot). Dragging a selected pin moves them all; E / S rotate / scale the selection around the pivot.
Every viewer edit runs in one edit block (opened at its first change, closed on release / when E / S ends),
so it is one undo step in the host.

**Add Pins By** chooses how pins are added in the viewer: double-click on the image (default),
single click, or only with the Add Pin button. A single click on the image always just
selects/deselects. Drags move a pin relative to where it was grabbed.

**Deformers** (up to 8) move pins procedurally before the mesh is solved. Each has a Type and
a Pins list (pin numbers separated by commas):

| Type | Pins | Controls |
| --- | --- | --- |
| Follower | leader, then followers (`4, 3`) | Follow Amount, Delay (frames) |
| Wave | base, then the chain outwards (`5, 6, 7`) | Bend, Swing, Speed, Lag per Joint, Phase |
| IK | root, joint, end (`hip, knee, foot`) | Flip Bend |
| FK Chain | base, then the chain outwards | Root / Joint 2 / Joint 3+ Angle |
| Wiggle | any pins | Size (px), Speed, Rotation (radius pins), Seed |
| Wind | base, then the chain outwards | Direction, Strength, Gustiness, Speed, Lag per Joint, Seed |
| Spring | leader, then followers | Follow Amount, Firmness, Damping (simulated from Simulation Start) |

Follower (with a delay) and Spring read their leader at other times through `pp::PinsAtTime`: its own
animation, run through the deformers above them that reach it, so a Spring can trail a pin a Wave swings.
The plugin memoises the raw pin reads per evaluation (`PinMemo`).

Wave, FK Chain and Wind chains may branch: each pin after the first hangs off the closest pin listed
before it (rest positions, `pp::chainParents`), so a whole skeleton fits in one list and each joint
turns only what hangs off it; Lag per Joint and the FK joint angles go by tree depth.
With IK, move the end pin and the joint pin bends to keep both bone lengths (the joint is
drawn pale: it is driven). Rig bones are drawn in the overlay. All deformer values can be
keyframed.

**Solver** sets how the image follows the pins over time:

| Solver | What it does |
| --- | --- |
| Static | Every frame is posed on its own (default) |
| Smooth Follow | The image lags behind the pins and eases in, no bounce |
| Elastic (Jiggle) | The image lags, overshoots and wobbles, then settles |

Areas at the pins always follow them exactly; the jiggle fades in with the distance from the
pins (**Jiggle Reach**, a share of the mesh size). **Firmness** is wobbles per second,
**Damping** how fast the wobble dies down (1 = no overshoot). The motion is simulated frame by
frame from **Simulation Start** (first frame of the clip, or a custom frame), so any frame
renders the same result whatever was viewed before, and the motion keeps going after the last
keyframe. Internally every vertex follows the pinned pose through an implicit damped spring;
the lag a point may build up is limited by its distance from the pins, so the mesh never folds
where the lag fades in.

**Weight Map** (Fusion page): connect a mask or matte to the node's *Weight Map* input.
*Use Weight Map To* **Anchor**: white areas stay exactly in place, grey areas keep that share of
their motion out. **Stiffen**: white areas keep their shape (and jiggle less) but still move.
*Read From* luminance or alpha; *Invert* uses black instead of white.

On the image, a selected pin shows a white diamond (drag it to set the pin's radius) and, with a radius, a
blue knob on the radius circle (drag it around to rotate the pin).

**Loop Pin Animation** (Off / Loop / Ping-Pong, Loop Start / End) remaps the time pin values are read at:
after Loop End the keys repeat or play back and forth; edits on a looped frame land inside the loop.

**Locators** (up to 8): Add Locator, drag its pink diamond onto a point of the image, Bake Locators. Baking
writes `locN_pos` / `locN_rot` keyframes for every frame of the clip, which other Fusion nodes can follow
(expression `PuppetPin1.loc1_pos`).

**Image layers** (Fusion page): inputs Layer 2..16 next to the Source. Each connected layer is meshed from its
own alpha and appended to one mesh (`Mesh::triSource`); later layers draw on top. Point pins bind to the top
layer under them, radius pins lock every layer inside the radius. **Output**: Composite, Single Layer, or
Motion Vectors (red/green = pixels moved since the previous frame, rendered by interpolating per-vertex
motion).

**Pin Mode > Edit Rest Pose** shows the original image so you can move where pins grab it.
**Mesh** (Triangles, Expansion, Alpha Threshold, Mesh From), **Solver** (Rigidity,
Iterations and the motion settings above) and **Render** (Antialiasing, Keep Unmeshed Pixels,
Draw Overlay Into Image) are in their own groups. *Draw Overlay Into Image* shows mesh and pins
in the output, for places where the overlay is not available. Antialiasing only smooths the
mesh edges; the image itself is sampled once per pixel, so undeformed areas stay pixel-exact.

## Build

Needs [llvm-mingw](https://github.com/mstorsjo/llvm-mingw) on a short path (its linker fails on
paths near 260 characters); run from a short path too.

```bash
LLVM=/c/path/to/llvm-mingw ./build.sh test   # plugin + core tests + OFX host tests
powershell -ExecutionPolicy Bypass -File install.ps1   # copies the bundle (asks for admin)
```

`tests/core_test.cpp` checks mesh coverage, pin accuracy and exact identity renders and writes
PNGs to `tests/out`. `tests/ofx_host_test.cpp` is a minimal OFX host that loads the built plugin,
clicks pins in through the overlay, keyframes, renders and draws the overlay.

Debug log: create an empty file `%TEMP%\puppetpin_debug`; the plugin then logs to
`%TEMP%\puppetpin_log.txt`.
