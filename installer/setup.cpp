// Puppet Pin Setup: a single-file installer for the Puppet Pin OFX plugin.
//
// The plugin binary is embedded as a resource. Setup asks for administrator rights (the OFX
// plugin folder is under Program Files), waits for DaVinci Resolve to be closed, and copies
// PuppetPin.ofx.bundle into "Common Files\OFX\Plugins".
//
//   PuppetPin-Setup.exe                 install / update (asks before replacing)
//   PuppetPin-Setup.exe /uninstall      remove
//   PuppetPin-Setup.exe /silent         no questions (still waits for Resolve to close)
//   PuppetPin-Setup.exe /target <dir>   install into <dir> instead (no admin; for testing)
#include <windows.h>
#include <shlobj.h>
#include <tlhelp32.h>

#include <string>
#include <vector>

#define PP_VERSION_TEXT L"1.5"

namespace {

const wchar_t* kTitle = L"Puppet Pin " PP_VERSION_TEXT L" Setup";
const int kPluginResource = 101;

bool resolveRunning()
{
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    if (snap == INVALID_HANDLE_VALUE) return false;
    PROCESSENTRY32W pe;
    pe.dwSize = sizeof pe;
    bool found = false;
    for (BOOL ok = Process32FirstW(snap, &pe); ok; ok = Process32NextW(snap, &pe))
        if (lstrcmpiW(pe.szExeFile, L"Resolve.exe") == 0) {
            found = true;
            break;
        }
    CloseHandle(snap);
    return found;
}

std::wstring defaultTarget()
{
    PWSTR common = nullptr;
    std::wstring dir = L"C:\\Program Files\\Common Files";
    if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_ProgramFilesCommon, 0, nullptr, &common)) && common) dir = common;
    if (common) CoTaskMemFree(common);
    return dir + L"\\OFX\\Plugins";
}

bool isElevated()
{
    HANDLE token = nullptr;
    if (!OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY, &token)) return false;
    TOKEN_ELEVATION elev{};
    DWORD size = 0;
    const bool ok = GetTokenInformation(token, TokenElevation, &elev, sizeof elev, &size) && elev.TokenIsElevated;
    CloseHandle(token);
    return ok;
}

// Runs this program again with administrator rights and returns its exit code.
int relaunchElevated(const std::wstring& args)
{
    wchar_t self[MAX_PATH * 4];
    GetModuleFileNameW(nullptr, self, (DWORD)(sizeof self / sizeof self[0]));
    SHELLEXECUTEINFOW sei{};
    sei.cbSize = sizeof sei;
    sei.fMask = SEE_MASK_NOCLOSEPROCESS;
    sei.lpVerb = L"runas";
    sei.lpFile = self;
    sei.lpParameters = args.c_str();
    sei.nShow = SW_SHOWNORMAL;
    if (!ShellExecuteExW(&sei)) {
        if (GetLastError() == ERROR_CANCELLED)
            MessageBoxW(nullptr, L"Setup needs administrator rights to install into the OFX plugin folder.", kTitle,
                        MB_ICONWARNING);
        return 5;
    }
    WaitForSingleObject(sei.hProcess, INFINITE);
    DWORD code = 1;
    GetExitCodeProcess(sei.hProcess, &code);
    CloseHandle(sei.hProcess);
    return (int)code;
}

bool exists(const std::wstring& p) { return GetFileAttributesW(p.c_str()) != INVALID_FILE_ATTRIBUTES; }

// Deletes a file or folder tree. A junction/symlink is removed as a link (its target is kept).
bool removeTree(const std::wstring& path)
{
    const DWORD attr = GetFileAttributesW(path.c_str());
    if (attr == INVALID_FILE_ATTRIBUTES) return true;
    if (!(attr & FILE_ATTRIBUTE_DIRECTORY)) {
        SetFileAttributesW(path.c_str(), FILE_ATTRIBUTE_NORMAL);
        return DeleteFileW(path.c_str()) != 0;
    }
    if (attr & FILE_ATTRIBUTE_REPARSE_POINT) return RemoveDirectoryW(path.c_str()) != 0;
    WIN32_FIND_DATAW fd;
    HANDLE h = FindFirstFileW((path + L"\\*").c_str(), &fd);
    if (h != INVALID_HANDLE_VALUE) {
        do {
            const std::wstring name = fd.cFileName;
            if (name == L"." || name == L"..") continue;
            removeTree(path + L"\\" + name);
        } while (FindNextFileW(h, &fd));
        FindClose(h);
    }
    return RemoveDirectoryW(path.c_str()) != 0;
}

bool writePlugin(const std::wstring& file)
{
    HRSRC res = FindResourceW(nullptr, MAKEINTRESOURCEW(kPluginResource), (LPCWSTR)RT_RCDATA);
    if (!res) return false;
    HGLOBAL data = LoadResource(nullptr, res);
    const DWORD size = SizeofResource(nullptr, res);
    const void* bytes = data ? LockResource(data) : nullptr;
    if (!bytes || size == 0) return false;
    const std::wstring tmp = file + L".new";
    HANDLE f = CreateFileW(tmp.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (f == INVALID_HANDLE_VALUE) return false;
    DWORD written = 0;
    const bool ok = WriteFile(f, bytes, size, &written, nullptr) && written == size;
    CloseHandle(f);
    if (!ok) {
        DeleteFileW(tmp.c_str());
        return false;
    }
    return MoveFileExW(tmp.c_str(), file.c_str(), MOVEFILE_REPLACE_EXISTING) != 0;
}

std::wstring lastErrorText()
{
    wchar_t* msg = nullptr;
    FormatMessageW(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, nullptr,
                   GetLastError(), 0, (LPWSTR)&msg, 0, nullptr);
    std::wstring s = msg ? msg : L"unknown error";
    if (msg) LocalFree(msg);
    return s;
}

int install(const std::wstring& target, bool silent)
{
    const std::wstring bundle = target + L"\\PuppetPin.ofx.bundle";
    const std::wstring bin = bundle + L"\\Contents\\Win64";
    if (exists(bundle) && !removeTree(bundle)) {
        if (!silent)
            MessageBoxW(nullptr, (L"Could not remove the old version:\n" + bundle + L"\n\n" + lastErrorText()).c_str(),
                        kTitle, MB_ICONERROR);
        return 3;
    }
    const int rc = SHCreateDirectoryExW(nullptr, bin.c_str(), nullptr);
    if (rc != ERROR_SUCCESS && rc != ERROR_ALREADY_EXISTS) {
        if (!silent) MessageBoxW(nullptr, (L"Could not create\n" + bin).c_str(), kTitle, MB_ICONERROR);
        return 3;
    }
    if (!writePlugin(bin + L"\\PuppetPin.ofx")) {
        if (!silent)
            MessageBoxW(nullptr, (L"Could not write the plugin:\n" + bin + L"\n\n" + lastErrorText()).c_str(), kTitle,
                        MB_ICONERROR);
        return 3;
    }
    if (!silent)
        MessageBoxW(nullptr,
                    L"Puppet Pin " PP_VERSION_TEXT L" is installed.\n\n"
                    L"Start DaVinci Resolve and find it under Effects > Open FX > Filters > Puppet Pin "
                    L"(Edit page), or add it as a node on the Fusion page (Shift+Space, \"Puppet Pin\").\n\n"
                    L"In the viewer, choose the \"Open FX Overlay\" on-screen controls, then double-click "
                    L"on the image to add pins and drag them to pose.",
                    kTitle, MB_ICONINFORMATION);
    return 0;
}

int uninstall(const std::wstring& target, bool silent)
{
    const std::wstring bundle = target + L"\\PuppetPin.ofx.bundle";
    if (!exists(bundle)) {
        if (!silent) MessageBoxW(nullptr, L"Puppet Pin is not installed.", kTitle, MB_ICONINFORMATION);
        return 0;
    }
    if (!removeTree(bundle)) {
        if (!silent)
            MessageBoxW(nullptr, (L"Could not remove\n" + bundle + L"\n\n" + lastErrorText()).c_str(), kTitle,
                        MB_ICONERROR);
        return 3;
    }
    if (!silent) MessageBoxW(nullptr, L"Puppet Pin was removed.", kTitle, MB_ICONINFORMATION);
    return 0;
}

}  // namespace

int WINAPI wWinMain(HINSTANCE, HINSTANCE, PWSTR, int)
{
    int argc = 0;
    LPWSTR* argv = CommandLineToArgvW(GetCommandLineW(), &argc);
    bool remove = false, silent = false;
    std::wstring target;
    for (int i = 1; argv && i < argc; ++i) {
        const std::wstring a = argv[i];
        if (lstrcmpiW(a.c_str(), L"/uninstall") == 0 || lstrcmpiW(a.c_str(), L"-uninstall") == 0) remove = true;
        else if (lstrcmpiW(a.c_str(), L"/silent") == 0 || lstrcmpiW(a.c_str(), L"/s") == 0) silent = true;
        else if ((lstrcmpiW(a.c_str(), L"/target") == 0) && i + 1 < argc) target = argv[++i];
    }
    if (argv) LocalFree(argv);
    const bool customTarget = !target.empty();
    if (!customTarget) target = defaultTarget();

    if (!customTarget && !isElevated()) {
        std::wstring args;
        if (remove) args += L" /uninstall";
        if (silent) args += L" /silent";
        return relaunchElevated(args);
    }

    while (!customTarget && resolveRunning()) {
        if (silent) {
            Sleep(1000);
            continue;
        }
        if (MessageBoxW(nullptr, L"DaVinci Resolve is running. Please close it, then press Retry.", kTitle,
                        MB_RETRYCANCEL | MB_ICONWARNING) != IDRETRY)
            return 4;
    }

    if (remove) return uninstall(target, silent);

    if (!silent && exists(target + L"\\PuppetPin.ofx.bundle")) {
        const int r = MessageBoxW(nullptr,
                                  L"Puppet Pin is already installed.\n\n"
                                  L"Yes: install version " PP_VERSION_TEXT L" (update / repair)\n"
                                  L"No: uninstall Puppet Pin\n"
                                  L"Cancel: do nothing",
                                  kTitle, MB_YESNOCANCEL | MB_ICONQUESTION);
        if (r == IDNO) return uninstall(target, silent);
        if (r != IDYES) return 0;
    }
    return install(target, silent);
}
