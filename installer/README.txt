PUPPET PIN 1.8  -  puppet pin warping for DaVinci Resolve (Windows)
=====================================================================

Bend and animate images with pins, like the After Effects puppet tool. Puppet Pin builds
a mesh from your image's transparency and bends it as rigidly as possible, so arms and
legs bend instead of smearing. Separate parts of the image (a ball next to a character,
for example) get their own mesh and move separately.

Works on the Edit page (as an Open FX effect) and on the Fusion page (as a node).
Requires: Windows 10 or 11 (64-bit), DaVinci Resolve or DaVinci Resolve Studio.
Tested with DaVinci Resolve Studio 21.


NEW IN 1.8
----------
* Fusion page: "Puppet Pin - Add Room". When an arm or leg pulled past the edge of the
  character image gets cut off, right-click the Puppet Pin node > Scripts > Puppet Pin -
  Add Room. One click gives Puppet Pin a bigger see-through picture to work in; the pose,
  keyframes, paths and expressions stay exactly as they were. Ctrl+Z undoes it.
* The viewer tells you when part of the puppet is past the edge and being cut off.

NEW IN 1.7
----------
* Fusion page: Puppet Pin always reads the whole Source, whatever part of the picture
  Fusion asks for, and tells hosts that allow it how far the posed puppet reaches.
* How to stop limbs being cut off on the Fusion page - see "ROOM TO MOVE" in
  QUICK START - FUSION PAGE.
* The viewer warns when the Source has no transparent background (for example a
  background merged in BEFORE Puppet Pin): then the whole frame is the puppet.

NEW IN 1.6
----------
* Onion Skin: outlines of the pose a few frames before (orange) and after (blue).
* Motion Trails: the path of the selected pins, a dot per frame, diamonds on keyframes.
* Pose tools: Key All Pins, Copy Pose / Paste Pose (also between Puppet Pin nodes) and
  Mirror Pose (the left arm's pose onto the right arm, or the whole pose flipped).
  All in the Animation tab, see ANIMATION TOOLS.
* Spring and Follower now also react to deformers above them: a Wave on the chest can
  make the hands bounce on a Spring, no keyframes needed.
* A pin's Layer can be keyframed: an arm swings behind the body, then in front.
* Motion Blur (Render tab): built in, no extra nodes.
* Save / Load Rig: pins, deformers, mesh and solver settings to a file, to reuse on
  another clip.

NEW IN 1.5
----------
* Select several pins: drag a box around them (start on an empty spot) or Shift+click
  them. Drag one to move them all; E rotates and S scales them around one pin - e.g. a
  whole arm around the shoulder. See SELECTING SEVERAL PINS.
* Fixed: Ctrl+Z after moving or rotating a pin sometimes undid an earlier step (like
  adding a pin). Every move, turn or scale in the viewer is now one undo step.
* Wave, Wind and FK Chain take a whole skeleton in one list: each pin hangs off the
  closest pin listed before it (see DEFORMERS), so the body sways instead of flinging.
* Overlapping parts no longer show through each other in stripes: a whole part is drawn
  on top - hands, feet and head over the body, unless the pins' Layer says otherwise.
* Tidier Inspector: every part has its own tab - Pins, Mesh, Solver, Deformers,
  Animation, Locators, Weight Map, Render (in the Fusion page).
* Fixed: changing Triangles, Expansion or Alpha Threshold before adding the first pin did
  not update the mesh shown in the viewer.
* Delete / Backspace removes the selected pin; E turns it and S scales it with the mouse.
* A pin whose settings change (also when Fusion's own path handle drags an animated pin)
  becomes the selected pin, so its rotate / radius handles show.
* Scale or Rotation on a pin without a radius gives it a starting radius.
* Dragging a pin only sets keyframes on pins (settings) you have keyframed yourself;
  "Keyframes When Dragging: Every Pin" brings back keyframing every drag.

NEW IN 1.4
----------
* Turn pins right on the image: a selected pin shows a white diamond below it - drag it to
  give the pin a radius - and a blue knob on the radius circle - drag it around to rotate.
* New deformers: Wiggle (pins wander around randomly), Wind (a chain leans into the wind,
  with gusts) and Spring (followers trail their leader on a spring).
* Locators: track a point of the image (a hat on a head) and bake its position and
  rotation, so other Fusion nodes can follow it.
* Image layers (Fusion page): up to 16 separate images (arm, body, head...) rigged
  together in one node, each with its own mesh, stacked in input order.
* Output: Composite, a Single Layer, or Motion Vectors (for Vector Motion Blur).
* Loop Pin Animation: repeat the pin keyframes (Loop) or play them back and forth
  (Ping-Pong).
* 48 pins (was 32).

NEW IN 1.3
----------
* Pin Influence: give a pin a reach (in pixels) and it only bends the image within it -
  drag a hand and only the arm moves, the rest stays put. Influence Falloff sets how
  soft the edge is. In the viewer, drag the green handle on the pin's circle.
* Live drag preview: while you drag a pin, the viewer shows the new pose right away (a
  slightly simplified version of the image). The full-quality frame appears as soon as
  you let go - Resolve only re-renders plugin effects after the mouse is released.

NEW IN 1.2
----------
* Solver: Smooth Follow and Elastic (Jiggle) - the image lags behind the pins, overshoots
  and wobbles, then settles. No extra keyframes needed.
* Weight Map input (Fusion page): hold areas still (Anchor) or keep them rigid (Stiffen)
  with a mask.
* Sharper output: antialiasing now only smooths the mesh edges, the image itself is no
  longer softened.
* A point pin pulled very far no longer stretches the image into a spike.


INSTALL
-------
1. Close DaVinci Resolve.
2. Unzip PuppetPin-1.8-Windows.zip (right-click > Extract All).
3. Double-click Install.bat in the unzipped folder and allow the administrator prompt.
   It copies the PuppetPin.ofx.bundle folder into
   C:\Program Files\Common Files\OFX\Plugins, where Resolve looks for Open FX plugins.
   If Windows says "Windows protected your PC", click "More info", then "Run anyway".
4. Start Resolve.

Rather do it by hand? Copy the PuppetPin.ofx.bundle folder into
C:\Program Files\Common Files\OFX\Plugins yourself (Windows asks for administrator
permission - click Continue).

To update: close Resolve and run Install.bat from the new zip (it replaces the old
version). To uninstall: run Uninstall.bat, or delete the PuppetPin.ofx.bundle folder from
C:\Program Files\Common Files\OFX\Plugins.


QUICK START - EDIT PAGE
-----------------------
1. Put an image with transparency on the timeline (a PNG cut-out of a character works
   best). Ordinary video works too: then the whole frame becomes the mesh.
2. Effects > Open FX > Filters > Puppet Pin, drag it onto the clip.
3. In the viewer's bottom-left on-screen controls menu choose "Open FX Overlay".
4. Double-click on the image to add a pin. Add pins on the parts that should stay put
   (feet, hips) and on the parts you want to move (hands, head).
5. Drag pins to pose. To animate a pin, give it a first keyframe yourself (the keyframe
   button next to its Position, under Pins > Pin N). From then on every drag on another
   frame sets a keyframe there: move the playhead, drag again, and the pose animates.
   Pins without keyframes just move. (Animation > Keyframes When Dragging: "Every Pin"
   keyframes every drag instead.)

QUICK START - FUSION PAGE
-------------------------
Add the node with Shift+Space and type "Puppet Pin" (or Effects > Open FX). Connect your
image, view the node and select it: the same double-click / drag controls work in the
viewer. The pin positions are normal Fusion controls, so you can keyframe them, edit
them in the Spline editor, "Connect To" a tracker, or drive them with expressions.

ROOM TO MOVE: on the Fusion page Puppet Pin's picture is exactly as big as the image
plugged into it, and Resolve does not let an Open FX plugin make it bigger. So an arm or
leg pulled past the edge of the character image is cut off (the viewer says so). Fix it in
one click: right-click the Puppet Pin node > Scripts > Puppet Pin - Add Room. It puts a
bigger, see-through picture in front of Puppet Pin (a transparent Background and a Merge)
and keeps the pose exactly as it was; run it again for even more room. Doing it by hand
works too: Merge the character over a transparent Background bigger than the character,
then into Puppet Pin (new pins then belong to the bigger picture).

The order matters:

    Character (with alpha) --> Puppet Pin --> Merge (Foreground)
    Background ------------------------------> Merge (Background)

* Puppet Pin goes on the cut-out character, the background is merged AFTER it. If the
  Merge comes first, the background is part of Puppet Pin's image, so the whole frame
  bends with the pins (the viewer then shows a yellow warning).
* Place and drag pins while viewing the Puppet Pin node itself. Viewing the Merge works
  only when the character fills the same frame there: Fusion draws the pins in the
  viewed node's frame, so a Merge that moves, scales or centres a different-sized
  character puts the pins off the character. Position the character with the Merge's
  Center / Size after posing, or view Puppet Pin while you work.


PINS
----
Click a pin to select it; its settings are on the Pins tab under "Pin N".

  Position / Rest Position   Where the pin is now / which point of the original image it
                             holds (0..1 of the frame).
  Radius                     Pixels around the pin that move rigidly with it. Needed for
                             Rotation and Scale. A big radius locks a whole area (a foot).
  Influence                  How far the pin pulls the image, in pixels measured along the
                             shape (so a hand pin does not pull the face next to it).
                             Beyond it the image stays as it would be without this pin.
                             0 = no limit (the whole image follows, as before). When it is
                             set, a green dotted circle shows it in the viewer: drag the
                             green square on the circle to resize it.
  Influence Falloff          How soft the edge of the influence is. The inner area moves
                             with the pin, the outer ring (this share of the influence)
                             stretches. 0 = hard edge.
  Rotation, Scale            Turn / scale the area inside the radius. On the image: a
                             selected pin shows a white diamond below it - drag it out to
                             give the pin a radius (drag it back onto the pin to remove the
                             radius) - and a blue knob on its radius circle: drag the knob
                             around the pin to rotate.
  Strength                   Lower values pull softer.
  Layer                      Where parts overlap (a tail behind or in front of the body),
                             the part around the pin with the higher layer is on top.
                             With equal layers, parts further out from the middle of the
                             body (hands, feet, head) go over the parts nearer to it.
                             Can be keyframed (an arm behind the body, then in front).

Buttons: Add Pin (adds a pin in the middle of the frame), Remove Selected Pins, Reset
Selected Pins (back to their rest positions, keyframes removed), Remove All Pins.
Delete (or Backspace) in the viewer removes the selected pins too.
E turns the selected pin: press E, then move the mouse around the pin (or drag anywhere);
click or press E again to keep the rotation, Esc to cancel. S scales the same way: press S
and move the mouse away from the pin (bigger) or towards it (smaller). (With no pin
selected, E and S are left to Resolve.)

Setting Scale or Rotation on a pin without a radius gives it a starting radius (they only
act on the area inside the radius); adjust it with the white diamond.

Add Pins By: Double-Click on Image (default), Single Click on Image, or Add Pin Button
Only. A single click on the image always just selects/deselects.

SELECTING SEVERAL PINS
* Drag a box from an empty spot: the pins inside it get selected. (With "Single Click on
  Image", start the box off the image, or hold Shift.)
* Shift+click (or Ctrl+click) a pin to add it to the selection or take it out.
* Or type the pin numbers into "Selected Pins" at the top, e.g.  8, 9, 10
The red pin is the pivot; the other selected pins are light red. After a box selection the
pivot is the pin where the group hangs on the rest of the body (the shoulder of an arm).
Click another selected pin to make it the pivot.
* Drag any selected pin: they all move together.
* E: they all turn around the pivot (move the mouse around it, click or E to keep, Esc to
  cancel). S: they all scale away from / towards the pivot. Pins with a radius also turn /
  scale their own area.
* Delete removes them all. A click on an empty spot deselects.

Pin Mode > Edit Rest Pose shows the original image: drag pins there to change where they
grab the image without changing the pose.

Up to 48 pins.

Loop Pin Animation (Animation tab): Off, Loop or Ping-Pong. After Loop End, the pin keyframes from Loop
Start to Loop End repeat (Loop) or play back and forth (Ping-Pong) - a walk cycle or a wave
keyed once runs forever. Dragging a pin on a looped frame edits the matching frame inside
the loop.


ANIMATION TOOLS (Animation tab)
-------------------------------
  Key All Pins     A position keyframe on every pin at this frame (rotation and scale
                   too where a pin uses them) - block out a pose in one click.
  Copy Pose        Remembers every pin's position, rotation and scale at this frame.
  Paste Pose       Puts the copied pose on the pins at this frame - to hold a pose, go
                   back to it, or paste it into another Puppet Pin node. With several
                   pins selected only those get it (paste just an arm). Keyframes are set
                   like when dragging (Keyframes When Dragging).
  Mirror Pose      Each selected pin's pose (how it moved away from its rest point,
                   mirrored left / right) goes onto its partner on the other side of the
                   body: select the left arm's pins and the right arm gets the same pose.
                   Partners are found from the rest points (the image should be roughly
                   symmetric); a pin in the middle mirrors itself. Nothing selected: the
                   whole pose flips.
  Onion Skin       Outlines of the pose a few frames before (orange) and after (blue).
                   Frames Before / Frames After: how many, Frame Step: frames between.
  Motion Trails    The path of the selected pins over the frames around this one: a dot
                   per frame (orange before, blue after), white diamonds on keyframes.
                   Trail Length: frames before and after.
Onion skin and trails are only drawn in the viewer.


SAVE / LOAD RIG
---------------
Save Rig writes the pins (their rest points, current positions and settings, without
keyframes), the deformers and the mesh / solver settings to the Rig File (empty =
Documents\PuppetPin Rigs\Rig.pprig). Load Rig replaces every pin and deformer with the
file's. Pin points are stored relative to the frame, so the rig fits a similar picture in
the same spot; move single pins in Pin Mode > Edit Rest Pose to fit.


SOLVER - STATIC, SMOOTH FOLLOW, ELASTIC (JIGGLE)
------------------------------------------------
Solver (on the Solver tab) sets how the image follows the pins over time:

  Static             Every frame is posed on its own (default).
  Smooth Follow      The image lags behind the pins and eases into place, no bounce.
  Elastic (Jiggle)   The image lags, overshoots and wobbles, then settles - antennas,
                     ears, hair, bellies, cloth, bouncy text.

  Firmness           Wobbles per second. Low = soft and slow, high = tight and quick.
  Damping            How fast the wobble dies down. 0 = wobbles for a long time,
                     1 = no overshoot at all.
  Jiggle Reach       How far from the pins the jiggle fades in. The area at a pin always
                     follows it exactly; free ends far from pins jiggle the most.
  Simulation Start   The frame where the motion starts at rest: the first frame of the
                     clip, or a custom Start Frame.

Animate the pins as usual (keyframes, a tracker, or a deformer). The jiggle keeps going
after the last keyframe until it settles. The motion is worked out frame by frame from the
start frame, so every frame always looks the same no matter where you jump in the timeline.
Tip: pin the part that is held (the base of an antenna, the head under the ears) and move
that pin - the free parts swing behind it.


WEIGHT MAP (Fusion page)
------------------------
The Puppet Pin node has a second input, "Weight Map". Connect a black-and-white image to
it: a Rectangle / Ellipse / Polygon mask, a painted matte, or a keyed image.

  Use Weight Map To   Anchor: white areas stay exactly where they are, grey areas keep
                      part of their motion out (50% grey moves half as far). Use it to
                      hold a background part or a hand resting on a table.
                      Stiffen: white areas keep their shape (no bending or stretching)
                      but still move with the pins - a hat, a phone, a sword. Stiffened
                      areas also jiggle less.
  Read From           Luminance or Alpha of the weight map.
  Invert              Use black instead of white.


DEFORMERS (rigs)
----------------
Deformer 1-8 move pins automatically before the image is bent. Choose a Type and enter
the pin numbers in "Pins", separated by commas (the numbers are shown next to the pins
in the viewer).

  Follower   Pins: leader first, then followers, e.g.  4, 3
             The followers copy the leader's motion. Follow Amount = how much,
             Delay = how many frames later (great for soft swaying of a body or head).

  Wave       Pins: base first, then the chain outwards, e.g.  5, 6, 7
             The chain swings around its base with a wave running down it - tails,
             capes, hair, without any keyframes. Bend = static bend, Swing = size,
             Speed = swings per second, Lag per Joint, Phase.

  IK         Pins: root, joint, end, e.g. hip, knee, foot:  1, 2, 3
             Move the end pin and the joint pin bends like a knee or elbow, keeping
             both bone lengths. The joint pin is drawn pale because the rig drives it.
             Flip Bend makes it bend the other way.

  FK Chain   Pins: base first, then the chain outwards.
             Pose the chain with Root Angle, Joint 2 Angle and Joint 3+ Angle.

  Wiggle     Pins: any, e.g.  5, 7, 9
             Each pin wanders around on its own, smoothly and randomly - idle motion,
             nervous shaking, floating. Size = how far (pixels), Speed = wiggles per
             second, Rotation = how much pins with a radius turn, Seed = another pattern.

  Wind       Pins: base first, then the chain outwards, e.g. a flag or plant: 1, 2, 3
             The chain leans into the wind and flutters with gusts that travel down it.
             Direction (0 = blows right, 90 = up, 180 = left), Strength = bend per joint,
             Gustiness (0 steady .. 1 very gusty), Speed, Lag per Joint, Seed.

  Spring     Pins: leader first, then followers, e.g. head, then ears:  8, 9, 10
             Like Follower, but on a spring: the followers lag behind the leader,
             overshoot and settle. Follow Amount, Firmness (wobbles per second),
             Damping (0 = wobbles long, 1 = no overshoot). Starts at Simulation Start.

Follower and Spring follow the leader's keyframes and every deformer above them that
moves it: Deformer 1 a Wave swaying the chest, Deformer 2 a Spring "chest, left hand,
right hand" - the hands bounce with the sway. (Deformers run from 1 to 8.)

Chains and skeletons (Wave, FK Chain, Wind): list the base first. Every following pin
hangs off the closest pin listed before it, so a plain chain (5, 6, 7) stays a chain and
a whole body can go in one list - hips, chest, head, then shoulder, elbow, hand, then the
other shoulder, elbow, hand:  1, 2, 3, 4, 5, 6, 7, 8, 9
Every joint bends what hangs off it. For a gentle body sway keep Swing small (2-5).

All deformer values can be keyframed. The rig is drawn in the viewer (blue bones,
pink dashed lines from a leader to its followers). Deformers can be combined with
Smooth Follow and Elastic.


LOCATORS
--------
A locator tracks one point of the image, for example to stick a hat to a head or let
particles come out of a hand.
1. Locators tab > Add Locator. It appears on the selected pin (or in the middle) as a pink
   diamond labelled L1.
2. Drag the diamond onto the point it should follow.
3. Locators tab > Bake Locators. This writes Position and Rotation keyframes for every frame of
   the clip (bake again after changing the animation).
In the Fusion page, other nodes can then follow it, e.g. an expression on a Transform's
Center:  PuppetPin1.loc1_pos   (and on its Angle:  PuppetPin1.loc1_rot).
Up to 8 locators. Untick Enabled to remove one.


IMAGE LAYERS (Fusion page)
--------------------------
Besides the Source, the node has inputs Layer 2 ... Layer 16. Put separate parts of a
character on separate inputs (arm, body, head, ...): each layer gets its own mesh, and
later layers are drawn on top (Source at the bottom, Layer 16 at the top). The same pins
work across all layers:
* A point pin grabs the top layer at that point.
* A pin with a Radius grabs every layer inside the radius - use that to join parts, e.g.
  a shoulder pin with a radius holds both the arm and the body.
"Mesh From: Full Frame" applies to the Source only; the other layers use their alpha.


OUTPUT (Render tab)
-------------------
  Composite (All Layers)  The posed image (default).
  Single Layer            Only one layer (Layer: 1 = Source, 2..16 = Layer inputs), to
                          grade, blur or shadow a part on its own.
  Motion Vectors          How far each pixel moved since the previous frame: red =
                          horizontal, green = vertical (pixels). Connect it to a Vector
                          Motion Blur node (vectors from red/green) for real motion blur.
  Motion Blur             Built-in blur of moving parts. Shutter: how long the shutter is
                          open in frames (0.5 = 180 degrees), Blur Quality: images averaged
                          per frame (more = smoother, slower).


MESH, SOLVER, RENDER SETTINGS
-----------------------------
Mesh:   Mesh From (Alpha, or Full Frame for opaque footage), Triangles (more = follows the
        outline closer and bends smoother), Expansion (how far the mesh reaches past the
        edge of the alpha), Alpha Threshold.
Solver: Rigidity (1 keeps every part's shape; lower lets parts stretch and squash),
        Iterations, and the motion settings above.
Render: Output (see above), Antialiasing (smooths the mesh edges), Keep Unmeshed Pixels
        (pixels outside the mesh stay where they are), Draw Overlay Into Image (shows mesh
        and pins in the output, e.g. for screenshots).


TIPS
----
* Pin everything that should not move. With only one pin, the whole part just slides.
* Parts that touch in the image are one mesh. If two legs are stuck together, separate
  them in the image (or lower Expansion / raise Triangles).
* For video with a moving subject, cut it out first (Magic Mask, Delta Keyer...) so the
  mesh follows the subject.
* Turn "Show Mesh" off in the Inspector to see the result without the mesh lines.
* The mesh, pins and circles are only drawn in the viewer, never in your render
  (unless "Draw Overlay Into Image" is turned on).
* Pulling one point very far bends the image around that point. To move a whole area
  (a hand, a foot) give its pin a Radius instead.


TROUBLESHOOTING
---------------
* No pins / mesh visible in the viewer: choose "Open FX Overlay" in the viewer's
  on-screen controls menu (Edit page), or select and view the node (Fusion page).
* "Puppet Pin" is missing from the effects: make sure Resolve was closed during the
  install and restart it. The plugin must be in
  C:\Program Files\Common Files\OFX\Plugins\PuppetPin.ofx.bundle\Contents\Win64\PuppetPin.ofx
* Double-clicking does not add a pin: the click has to be on the image (inside the mesh),
  or set "Add Pins By" to "Single Click on Image".
* Fusion page, an arm or leg is cut off with a straight edge when it moves far: Puppet
  Pin's picture is the size of its input. Right-click the Puppet Pin node > Scripts >
  Puppet Pin - Add Room (see ROOM TO MOVE in QUICK START - FUSION PAGE).
* Fusion page, the whole frame moves instead of the character: the Source has no
  transparent background. Connect the cut-out character to Puppet Pin and merge the
  background after it (see QUICK START - FUSION PAGE).
* Fusion page, pins sit next to the character when viewing a Merge: view the Puppet Pin
  node while placing and dragging pins.


LICENSE
-------
Puppet Pin is free and open source under the MIT license (see LICENSE.txt in this zip):
use it, change it and share it, as long as the copyright notice stays with it.

It includes the OpenFX support library:
Copyright (c) 2004-2007, The Open Effects Association Ltd. All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met: redistributions of source code
must retain the above copyright notice, this list of conditions and the following
disclaimer; redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or other materials
provided with the distribution; neither the name The Open Effects Association Ltd, nor the
names of its contributors may be used to endorse or promote products derived from this
software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
