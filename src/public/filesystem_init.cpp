//========= Copyright Valve Corporation, All rights reserved. ============//
//
// Purpose:
//
//=============================================================================

#undef PROTECTED_THINGS_ENABLE
#undef PROTECT_FILEIO_FUNCTIONS
#ifndef POSIX
#undef fopen
#endif

#if defined(_WIN32) && !defined(_X360)
#include <direct.h>
#include <io.h>
#include <process.h>
#include <windows.h>

#elif defined(POSIX)
#include <unistd.h>
#define _chdir chdir
#define _access access
#endif
#include "KeyValues.h"
#include "appframework/IAppSystemGroup.h"
#include "filesystem_init.h"
#include "tier0/icommandline.h"
#include "tier1/smartptr.h"
#include "tier1/strtools.h"
#include "tier1/utlbuffer.h"
#include <stdio.h>
#include <sys/stat.h>

#if defined(_X360)
#include "xbox\xbox_win32stubs.h"
#endif

#include "steam/steam_api.h"

// memdbgon must be the last include file in a .cpp file!!!
#include <tier0/memdbgon.h>

#if !defined(_X360)
#define GAMEINFO_FILENAME "gameinfo.txt"
#else
// The .xtx file is a TCR requirement, as .txt files cannot live on the DVD.
// The .xtx file only exists outside the zips (same as .txt and is made during
// the image build) and is read to setup the search paths. So all other code
// should be able to safely expect gameinfo.txt after the zip is mounted as the
// .txt file exists inside the zips. The .xtx concept is private and should only
// have to occurr here. As a safety measure, if the .xtx file is not found a
// retry is made with the original .txt name
#define GAMEINFO_FILENAME "gameinfo.xtx"
#endif
#define GAMEINFO_FILENAME_ALTERNATE "gameinfo.txt"

static char g_FileSystemError[256];
static bool s_bUseVProjectBinDir = false;
static FSErrorMode_t g_FileSystemErrorMode = FS_ERRORMODE_VCONFIG;

// Call this to use a bin directory relative to VPROJECT
void FileSystem_UseVProjectBinDir(bool bEnable) {
  s_bUseVProjectBinDir = bEnable;
}

bool FileSystem_AllowedSearchPath(const char *pchPath);

// This class lets you modify environment variables, and it restores the
// original value when it goes out of scope.
class CTempEnvVar {
public:
  CTempEnvVar(const char *pVarName) {
    m_bRestoreOriginalValue = true;
    m_pVarName = pVarName;

    const char *pValue = NULL;

#ifdef _WIN32
    // Use GetEnvironmentVariable instead of getenv because getenv doesn't pick
    // up changes to the process environment after the DLL was loaded.
    char szBuf[4096];
    if (GetEnvironmentVariable(m_pVarName, szBuf, sizeof(szBuf)) != 0) {
      pValue = szBuf;
    }
#else
    // LINUX BUG: see above
    pValue = getenv(pVarName);
#endif

    if (pValue) {
      m_bExisted = true;
      m_OriginalValue.SetSize(Q_strlen(pValue) + 1);
      memcpy(m_OriginalValue.Base(), pValue, m_OriginalValue.Count());
    } else {
      m_bExisted = false;
    }
  }

  ~CTempEnvVar() {
    if (m_bRestoreOriginalValue) {
      // Restore the original value.
      if (m_bExisted) {
        SetValue("%s", m_OriginalValue.Base());
      } else {
        ClearValue();
      }
    }
  }

  void SetRestoreOriginalValue(bool bRestore) {
    m_bRestoreOriginalValue = bRestore;
  }

  int GetValue(char *pszBuf, int nBufSize) {
    if (!pszBuf || (nBufSize <= 0))
      return 0;

#ifdef _WIN32
    // Use GetEnvironmentVariable instead of getenv because getenv doesn't pick
    // up changes to the process environment after the DLL was loaded.
    return GetEnvironmentVariable(m_pVarName, pszBuf, nBufSize);
#else
    // LINUX BUG: see above
    const char *pszOut = getenv(m_pVarName);
    if (!pszOut) {
      *pszBuf = '\0';
      return 0;
    }

    Q_strncpy(pszBuf, pszOut, nBufSize);
    return Q_strlen(pszBuf);
#endif
  }

  void SetValue(const char *pValue, ...) {
    char valueString[4096];
    va_list marker;
    va_start(marker, pValue);
    Q_vsnprintf(valueString, sizeof(valueString), pValue, marker);
    va_end(marker);

#ifdef WIN32
    char str[4096];
    Q_snprintf(str, sizeof(str), "%s=%s", m_pVarName, valueString);
    _putenv(str);
#else
    setenv(m_pVarName, valueString, 1);
#endif
  }

  void ClearValue() {
#ifdef WIN32
    char str[512];
    Q_snprintf(str, sizeof(str), "%s=", m_pVarName);
    _putenv(str);
#else
    setenv(m_pVarName, "", 1);
#endif
  }

private:
  bool m_bRestoreOriginalValue;
  const char *m_pVarName;
  bool m_bExisted;
  CUtlVector<char> m_OriginalValue;
};

class CSteamEnvVars {
public:
  CSteamEnvVars()
      : m_SteamAppId("SteamAppId"),
        m_SteamUserPassphrase("SteamUserPassphrase"),
        m_SteamAppUser("SteamAppUser"), m_Path("path") {}

  void SetRestoreOriginalValue_ALL(bool bRestore) {
    m_SteamAppId.SetRestoreOriginalValue(bRestore);
    m_SteamUserPassphrase.SetRestoreOriginalValue(bRestore);
    m_SteamAppUser.SetRestoreOriginalValue(bRestore);
    m_Path.SetRestoreOriginalValue(bRestore);
  }

  CTempEnvVar m_SteamAppId;
  CTempEnvVar m_SteamUserPassphrase;
  CTempEnvVar m_SteamAppUser;
  CTempEnvVar m_Path;
};

// ----------------------------------------------------------------------------------------------------
// // Helpers.
// ----------------------------------------------------------------------------------------------------
// //
void Q_getwd(char *out, int outSize) {
#if defined(_WIN32) || defined(WIN32)
  _getcwd(out, outSize);
  Q_strncat(out, "\\", outSize, COPY_ALL_CHARACTERS);
#else
  getcwd(out, outSize);
  strcat(out, "/");
#endif
  Q_FixSlashes(out);
}

// ----------------------------------------------------------------------------------------------------
// // Module interface.
// ----------------------------------------------------------------------------------------------------
// //

CFSSearchPathsInit::CFSSearchPathsInit() {
  m_pDirectoryName = NULL;
  m_pLanguage = NULL;
  m_ModPath[0] = 0;
  m_bMountHDContent = m_bLowViolence = false;
}

CFSSteamSetupInfo::CFSSteamSetupInfo() {
  m_pDirectoryName = NULL;
  m_bOnlyUseDirectoryName = false;
  m_bSteam = false;
  m_bToolsMode = true;
  m_bNoGameInfo = false;
}

CFSLoadModuleInfo::CFSLoadModuleInfo() {
  m_pFileSystemDLLName = NULL;
  m_pFileSystem = NULL;
  m_pModule = NULL;
}

CFSMountContentInfo::CFSMountContentInfo() {
  m_bToolsMode = true;
  m_pDirectoryName = NULL;
  m_pFileSystem = NULL;
}

const char *FileSystem_GetLastErrorString() { return g_FileSystemError; }

static bool GetAppInstallDirFromSteamVDF(int nAppId, char *szAppInstallDir, int nMaxLen);

KeyValues *ReadKeyValuesFile(const char *pFilename) {
  // Read in the gameinfo.txt file and null-terminate it.
  FILE *fp = fopen(pFilename, "rb");
  if (!fp)
    return NULL;
  CUtlVector<char> buf;
  fseek(fp, 0, SEEK_END);
  buf.SetSize(ftell(fp) + 1);
  fseek(fp, 0, SEEK_SET);
  fread(buf.Base(), 1, buf.Count() - 1, fp);
  fclose(fp);
  buf[buf.Count() - 1] = 0;

  KeyValues *kv = new KeyValues("");
  if (!kv->LoadFromBuffer(pFilename, buf.Base())) {
    kv->deleteThis();
    return NULL;
  }

  return kv;
}

static char szSDKExecDir[4096];
const char *GetSDKExecDir() {
#if defined(_WIN32)
  // misyl: In Windows, getenv doesn't update after the application has already
  // launched.
  if (GetEnvironmentVariableA("SDK_EXEC_DIR", szSDKExecDir,
                              sizeof(szSDKExecDir))) {
    if (*szSDKExecDir)
      return szSDKExecDir;
  }
#else
  const char *pszEnv = getenv("SDK_EXEC_DIR");
  if (pszEnv && *pszEnv) {
    V_strncpy(szSDKExecDir, pszEnv, sizeof(szSDKExecDir));
    return szSDKExecDir;
  }
#endif

  return nullptr;
}

static bool Sys_GetExecutableName(char *out, int len) {
#if defined(_WIN32)
  if (!::GetModuleFileName((HINSTANCE)GetModuleHandle(NULL), out, len)) {
    return false;
  }
#else
  if (CommandLine()->GetParm(0)) {
    Q_MakeAbsolutePath(out, len, CommandLine()->GetParm(0));
  } else {
    return false;
  }
#endif

  return true;
}

// Returns the actual directory of the running executable. Used for loading
// DLLs (filesystem_stdio.dll) that must always come from our own directory.
static bool FileSystem_GetRealExecutableDir(char *exedir, int exeDirLen) {
  if (!Sys_GetExecutableName(exedir, exeDirLen))
    return false;
  Q_StripFilename(exedir);
  Q_FixSlashes(exedir);
  return true;
}

// Helper: Given a root directory, probe for a bin directory containing
// filesystem_stdio.dll. Only checks bin\x64 (SDK standard layout).
// We intentionally do NOT probe bin\win64 (Garry's Mod layout) because
// Gmod ships incompatible DLLs (materialsystem.dll, shaderapiempty.dll).
static bool FileSystem_ProbeBinDir(const char *pRootDir, char *exedir,
                                   int exeDirLen) {
  Q_snprintf(exedir, exeDirLen, "%s%c" PLATFORM_BIN_DIR, pRootDir,
             CORRECT_PATH_SEPARATOR);
  Q_FixSlashes(exedir);

  char dllPath[MAX_PATH];
  Q_snprintf(dllPath, sizeof(dllPath), "%s%cfilesystem_stdio" DLL_EXT_STRING,
             exedir, CORRECT_PATH_SEPARATOR);
  if (_access(dllPath, 0) == 0)
    return true;

  return false;
}

bool FileSystem_GetExecutableDir(char *exedir, int exeDirLen) {
  exedir[0] = 0;

  // 1. If SDK_EXEC_DIR is set, use it (legacy support).
  const char *pSDKExecDir = GetSDKExecDir();
  if (pSDKExecDir) {
    V_strncpy(exedir, pSDKExecDir, exeDirLen);
    Q_FixSlashes(exedir);

    // Ensure the path ends with PLATFORM_BIN_DIR (bin\x64)
    char ext[MAX_PATH];
    Q_StrRight(exedir, V_strlen(PLATFORM_BIN_DIR) + 1, ext, sizeof(ext));
    if (ext[0] != CORRECT_PATH_SEPARATOR ||
        Q_stricmp(ext + 1, PLATFORM_BIN_DIR) != 0) {
      Q_strncat(exedir, CORRECT_PATH_SEPARATOR_S, exeDirLen,
                COPY_ALL_CHARACTERS);
      Q_strncat(exedir, PLATFORM_BIN_DIR, exeDirLen, COPY_ALL_CHARACTERS);
      Q_FixSlashes(exedir);
    }
#if defined(_WIN32)
    SetDllDirectoryA(exedir);
#endif
    return true;
  }

  // 2. If -binroot is specified, derive executable dir from it.
  const char *pContentRoot = CommandLine()->ParmValue("-binroot");
  if (pContentRoot) {
    char rootDir[MAX_PATH];
    Q_MakeAbsolutePath(rootDir, sizeof(rootDir), pContentRoot);
    Q_StripTrailingSlash(rootDir);
    if (FileSystem_ProbeBinDir(rootDir, exedir, exeDirLen)) {
#if defined(_WIN32)
      SetDllDirectoryA(exedir);
#endif
      return true;
    }
    // Contentroot specified but no bin dir found — fall through.
  }

  // 3. If -game is specified, derive executable dir from the game
  //    directory's parent. This allows running against games like
  //    Garry's Mod without needing -binroot.
  const char *pGameDir = GetVProjectCmdLineValue();
  if (pGameDir) {
    char rootDir[MAX_PATH];
    Q_MakeAbsolutePath(rootDir, sizeof(rootDir), pGameDir);
    Q_StripTrailingSlash(rootDir);
    Q_StripFilename(rootDir); // e.g. GarrysMod\garrysmod -> GarrysMod
    if (FileSystem_ProbeBinDir(rootDir, exedir, exeDirLen)) {
#if defined(_WIN32)
      SetDllDirectoryA(exedir);
#endif
      return true;
    }

    // -game specified but no bin dir found — it might be a standalone mod directory
    // (e.g. sourcemods\apawcalypse2063) lacking its own engine binaries. 
    // We will parse the mod's gameinfo.txt to find its base AppID, and mount that engine instead.
    char szGameInfoPath[MAX_PATH];
    Q_snprintf(szGameInfoPath, sizeof(szGameInfoPath), "%s\\gameinfo.txt", pGameDir);
    KeyValues *pGameInfo = ReadKeyValuesFile(szGameInfoPath);
    if (pGameInfo) {
      KeyValues *pFileSystem = pGameInfo->FindKey("FileSystem");
      if (pFileSystem) {
        int nSteamAppId = pFileSystem->GetInt("SteamAppId", -1);
        if (nSteamAppId != -1) {
          char szAppInstallDir[MAX_PATH];
          if (GetAppInstallDirFromSteamVDF(nSteamAppId, szAppInstallDir, sizeof(szAppInstallDir))) {
            if (FileSystem_ProbeBinDir(szAppInstallDir, exedir, exeDirLen)) {
#if defined(_WIN32)
              SetDllDirectoryA(exedir);
#endif
              pGameInfo->deleteThis();
              return true;
            }
          }
        }
      }
      pGameInfo->deleteThis();
    }
  }

  // 4. Fallback: use the actual executable's directory.
  bool bSuccess = FileSystem_GetRealExecutableDir(exedir, exeDirLen);
#if defined(_WIN32)
  if (bSuccess) {
    SetDllDirectoryA(exedir);
  }
#endif
  return bSuccess;
}

static bool FileSystem_GetBaseDir(char *baseDir, int baseDirLen) {
  // If -binroot was specified, use it directly as the base directory.
  // This replaces SDK_EXEC_DIR and allows the tool to resolve content
  // (VPKs, materials, models) from a specific game installation
  // regardless of where the executable or -game directory are located.
  const char *pContentRoot = CommandLine()->ParmValue("-binroot");
  if (pContentRoot) {
    Q_MakeAbsolutePath(baseDir, baseDirLen, pContentRoot);
    Q_StripTrailingSlash(baseDir);
    return true;
  }

  // If -game was specified, derive base dir from the game directory's parent.
  // This is the standard Source convention: -game points to a subdirectory
  // (e.g. "GarrysMod/garrysmod") and content paths in gameinfo.txt are
  // relative to the parent (e.g. "GarrysMod/").
  // This allows the tool to run from outside the game's bin directory.
  const char *pGameDir = GetVProjectCmdLineValue();
  if (pGameDir) {
    Q_MakeAbsolutePath(baseDir, baseDirLen, pGameDir);
    Q_StripTrailingSlash(baseDir);
    Q_StripFilename(baseDir);
    return true;
  }

  // Fallback: derive from executable location (original behavior)
  if (FileSystem_GetExecutableDir(baseDir, baseDirLen)) {
    Q_StripFilename(baseDir);
    if (PLATFORM_DIR[0] != '\0')
      Q_StripFilename(baseDir);
    return true;
  }

  return false;
}

void LaunchVConfig() {
#if defined(_WIN32) && !defined(_X360)
  char vconfigExe[MAX_PATH];
  FileSystem_GetExecutableDir(vconfigExe, sizeof(vconfigExe));
  Q_AppendSlash(vconfigExe, sizeof(vconfigExe));
  Q_strncat(vconfigExe, "vconfig.exe", sizeof(vconfigExe), COPY_ALL_CHARACTERS);

  char *argv[] = {vconfigExe, "-allowdebug", NULL};

  _spawnv(_P_NOWAIT, vconfigExe, argv);
#elif defined(_X360)
  Msg("Launching vconfig.exe not supported\n");
#endif
}

const char *GetVProjectCmdLineValue() {
  return CommandLine()->ParmValue("-vproject",
                                  CommandLine()->ParmValue("-game"));
}

FSReturnCode_t SetupFileSystemError(bool bRunVConfig, FSReturnCode_t retVal,
                                    const char *pMsg, ...) {
  va_list marker;
  va_start(marker, pMsg);
  Q_vsnprintf(g_FileSystemError, sizeof(g_FileSystemError), pMsg, marker);
  va_end(marker);

  Warning("%s\n", g_FileSystemError);

  // Run vconfig?
  // Don't do it if they specifically asked for it not to, or if they manually
  // specified a vconfig with -game or -vproject.
  if (bRunVConfig && g_FileSystemErrorMode == FS_ERRORMODE_VCONFIG &&
      !CommandLine()->FindParm(CMDLINEOPTION_NOVCONFIG) &&
      !GetVProjectCmdLineValue()) {
    LaunchVConfig();
  }

  if (g_FileSystemErrorMode == FS_ERRORMODE_AUTO ||
      g_FileSystemErrorMode == FS_ERRORMODE_VCONFIG) {
    Error("%s\n", g_FileSystemError);
  }

  return retVal;
}

FSReturnCode_t LoadGameInfoFile(const char *pDirectoryName,
                                KeyValues *&pMainFile,
                                KeyValues *&pFileSystemInfo,
                                KeyValues *&pSearchPaths) {
  // If GameInfo.txt exists under pBaseDir, then this is their game directory.
  // All the filesystem mappings will be in this file.
  char gameinfoFilename[MAX_PATH];
  Q_strncpy(gameinfoFilename, pDirectoryName, sizeof(gameinfoFilename));
  Q_AppendSlash(gameinfoFilename, sizeof(gameinfoFilename));
  Q_strncat(gameinfoFilename, GAMEINFO_FILENAME, sizeof(gameinfoFilename),
            COPY_ALL_CHARACTERS);
  Q_FixSlashes(gameinfoFilename);
  pMainFile = ReadKeyValuesFile(gameinfoFilename);
  if (IsX360() && !pMainFile) {
    // try again
    Q_strncpy(gameinfoFilename, pDirectoryName, sizeof(gameinfoFilename));
    Q_AppendSlash(gameinfoFilename, sizeof(gameinfoFilename));
    Q_strncat(gameinfoFilename, GAMEINFO_FILENAME_ALTERNATE,
              sizeof(gameinfoFilename), COPY_ALL_CHARACTERS);
    Q_FixSlashes(gameinfoFilename);
    pMainFile = ReadKeyValuesFile(gameinfoFilename);
  }
  if (!pMainFile) {
    return SetupFileSystemError(true, FS_MISSING_GAMEINFO_FILE,
                                "%s is missing.", gameinfoFilename);
  }

  pFileSystemInfo = pMainFile->FindKey("FileSystem");
  if (!pFileSystemInfo) {
    pMainFile->deleteThis();
    return SetupFileSystemError(true, FS_INVALID_GAMEINFO_FILE,
                                "%s is not a valid format.", gameinfoFilename);
  }

  // Now read in all the search paths.
  pSearchPaths = pFileSystemInfo->FindKey("SearchPaths");
  if (!pSearchPaths) {
    pMainFile->deleteThis();
    return SetupFileSystemError(true, FS_INVALID_GAMEINFO_FILE,
                                "%s is not a valid format.", gameinfoFilename);
  }
  return FS_OK;
}

static void FileSystem_AddLoadedSearchPath(CFSSearchPathsInit &initInfo,
                                           const char *pPathID,
                                           const char *fullLocationPath,
                                           bool bLowViolence) {

  // Check for mounting LV game content in LV builds only
  if (V_stricmp(pPathID, "game_lv") == 0) {

    // Not in LV build, don't mount
    if (!initInfo.m_bLowViolence)
      return;

    // Mount, as a game path
    pPathID = "game";
  }

  // Check for mounting HD game content if enabled
  if (V_stricmp(pPathID, "game_hd") == 0) {

    // Not in LV build, don't mount
    if (!initInfo.m_bMountHDContent)
      return;

    // Mount, as a game path
    pPathID = "game";
  }

  // Special processing for ordinary game folders
  if (V_stristr(fullLocationPath, ".vpk") == NULL &&
      Q_stricmp(pPathID, "game") == 0) {
    if (CommandLine()->FindParm("-tempcontent") != 0) {
      char szPath[MAX_PATH];
      Q_snprintf(szPath, sizeof(szPath), "%s_tempcontent", fullLocationPath);
      initInfo.m_pFileSystem->AddSearchPath(szPath, pPathID, PATH_ADD_TO_TAIL);
    }
  }

  if (initInfo.m_pLanguage && Q_stricmp(initInfo.m_pLanguage, "english") &&
      V_strstr(fullLocationPath, "_english") != NULL) {
    char szPath[MAX_PATH];
    char szLangString[MAX_PATH];

    // Need to add a language version of this path first

    Q_snprintf(szLangString, sizeof(szLangString), "_%s", initInfo.m_pLanguage);
    V_StrSubst(fullLocationPath, "_english", szLangString, szPath,
               sizeof(szPath), true);
    initInfo.m_pFileSystem->AddSearchPath(szPath, pPathID, PATH_ADD_TO_TAIL);
  }

  initInfo.m_pFileSystem->AddSearchPath(fullLocationPath, pPathID,
                                        PATH_ADD_TO_TAIL);
}

static int SortStricmp(char *const *sz1, char *const *sz2) {
  return V_stricmp(*sz1, *sz2);
}

#ifdef _WIN32
static void Plat_OpenURL(const char *pchURL) {
  ShellExecuteA(0, 0, pchURL, 0, 0, SW_SHOW);
}
#elif defined(LINUX)
static void Plat_OpenURL(const char *pchURL) {
  // from SteamVR
  if (!pchURL || !*pchURL)
    return;

  char szFullCommand[4096];

  V_sprintf_safe(szFullCommand, "xdg-open '%s' &", pchURL);

  int nResult = system(szFullCommand);
  if (nResult == -1) {
    Warning("Plat_OpenURL: Failed to execute '%s'\n", szFullCommand);
  }
}
#endif

struct Source1AppidInfo_t {
  uint32 nAppId;
  const char *pszName;
} g_Source1Appids[]{
    {220, "Half-Life 2"},
    {320, "Half-Life 2: Deathmatch"},
    {340, "Half-Life 2: Lost Coast"},
    // Deprecated.
    //{ 380, "Half-Life 2: Episode One" },
    //{ 420, "Half-Life 2: Episode Two" },

    {240, "Counter-Strike: Source"},
    {300, "Day of Defeat: Source"},

    {280, "Half-Life: Source"},
    {360, "Half-Life Deathmatch: Source"},

    {440, "Team Fortress 2"},

    {400, "Portal"},
    {620, "Portal 2"},
    {500, "Left 4 Dead"},
    {550, "Left 4 Dead 2"},
    {630, "Alien Swarm"},

    {243750, "Source SDK Base 2013 Multiplayer"},
    {243730, "Source SDK Base 2013 Singleplayer"},
    {218, "Source SDK Base 2007"},
    {215, "Source SDK Base 2006"},
    {211, "Source SDK (Legacy)"},
};

const Source1AppidInfo_t *GetKnownAppidInfo(uint32 nAppid) {
  for (int i = 0; i < ARRAYSIZE(g_Source1Appids); i++) {
    if (nAppid == g_Source1Appids[i].nAppId)
      return &g_Source1Appids[i];
  }

  return nullptr;
}

static bool GetAppInstallDirFromSteamVDF(int nAppId, char *szAppInstallDir, int nMaxLen) {
    char szSteamPath[MAX_PATH] = {0};

#if defined(_WIN32)
    HKEY hKey;
    if (RegOpenKeyExA(HKEY_CURRENT_USER, "Software\\Valve\\Steam", 0, KEY_READ, &hKey) == ERROR_SUCCESS) {
        DWORD dwType = REG_SZ;
        DWORD dwSize = sizeof(szSteamPath);
        if (RegQueryValueExA(hKey, "SteamPath", NULL, &dwType, (LPBYTE)szSteamPath, &dwSize) != ERROR_SUCCESS) {
            szSteamPath[0] = 0;
        }
        RegCloseKey(hKey);
    }
#elif defined(POSIX)
    const char *pHome = getenv("HOME");
    if (pHome) {
        // Try standard Linux paths
        Q_snprintf(szSteamPath, sizeof(szSteamPath), "%s/.local/share/Steam", pHome);
        if (_access(szSteamPath, 0) != 0) {
            Q_snprintf(szSteamPath, sizeof(szSteamPath), "%s/.steam/steam", pHome);
        }
    }
#endif

    if (!szSteamPath[0] || _access(szSteamPath, 0) != 0) {
        return false; // Steam not found
    }

    char szLibraryFolders[MAX_PATH];
    Q_snprintf(szLibraryFolders, sizeof(szLibraryFolders), "%s/steamapps/libraryfolders.vdf", szSteamPath);
    Q_FixSlashes(szLibraryFolders);

    KeyValues *pLibraryFolders = ReadKeyValuesFile(szLibraryFolders);
    if (!pLibraryFolders) {
        return false;
    }

    bool bFound = false;

    for (KeyValues *pLib = pLibraryFolders->GetFirstSubKey(); pLib != NULL; pLib = pLib->GetNextKey()) {
        const char *pszPath = pLib->GetString("path", NULL);
        if (!pszPath || !pszPath[0])
            continue;

        char szManifest[MAX_PATH];
        Q_snprintf(szManifest, sizeof(szManifest), "%s/steamapps/appmanifest_%d.acf", pszPath, nAppId);
        Q_FixSlashes(szManifest);

        if (_access(szManifest, 0) == 0) {
            KeyValues *pManifest = ReadKeyValuesFile(szManifest);
            if (pManifest) {
                const char *pszInstallDir = pManifest->GetString("installdir", NULL);
                if (pszInstallDir && pszInstallDir[0]) {
                    // Build the final path
                    Q_snprintf(szAppInstallDir, nMaxLen, "%s/steamapps/common/%s", pszPath, pszInstallDir);
                    Q_FixSlashes(szAppInstallDir);
                    bFound = true;
                }
                pManifest->deleteThis();
            }
        }

        if (bFound)
            break;
    }

    pLibraryFolders->deleteThis();
    return bFound;
}

FSReturnCode_t FileSystem_LoadSearchPaths(CFSSearchPathsInit &initInfo) {
  if (!initInfo.m_pFileSystem || !initInfo.m_pDirectoryName)
    return SetupFileSystemError(
        false, FS_INVALID_PARAMETERS,
        "FileSystem_LoadSearchPaths: Invalid parameters specified.");

  KeyValues *pMainFile, *pFileSystemInfo, *pSearchPaths;
  FSReturnCode_t retVal = LoadGameInfoFile(initInfo.m_pDirectoryName, pMainFile,
                                           pFileSystemInfo, pSearchPaths);
  if (retVal != FS_OK)
    return retVal;

  // All paths except those marked with |gameinfo_path| are relative to the base
  // dir.
  char baseDir[MAX_PATH];
  if (!FileSystem_GetBaseDir(baseDir, sizeof(baseDir)))
    return SetupFileSystemError(false, FS_INVALID_PARAMETERS,
                                "FileSystem_GetBaseDir failed.");

  // The MOD directory is always the one that contains gameinfo.txt
  Q_strncpy(initInfo.m_ModPath, initInfo.m_pDirectoryName,
            sizeof(initInfo.m_ModPath));

#define GAMEINFOPATH_TOKEN "|gameinfo_path|"
#define BASESOURCEPATHS_TOKEN "|all_source_engine_paths|"
#define APPID_PREFIX_TOKEN "|appid_"

  const char *pszExtraSearchPath =
      CommandLine()->ParmValue("-insert_search_path");
  if (pszExtraSearchPath) {
    CUtlStringList vecPaths;
    V_SplitString(pszExtraSearchPath, ",", vecPaths);
    FOR_EACH_VEC(vecPaths, idxExtraPath) {
      char szAbsSearchPath[MAX_PATH];
      Q_StripPrecedingAndTrailingWhitespace(vecPaths[idxExtraPath]);
      V_MakeAbsolutePath(szAbsSearchPath, sizeof(szAbsSearchPath),
                         vecPaths[idxExtraPath], baseDir);
      V_FixSlashes(szAbsSearchPath);
      if (!V_RemoveDotSlashes(szAbsSearchPath))
        Error("Bad -insert_search_path - Can't resolve pathname for '%s'",
              szAbsSearchPath);
      V_StripTrailingSlash(szAbsSearchPath);
      FileSystem_AddLoadedSearchPath(initInfo, "GAME", szAbsSearchPath, false);
      FileSystem_AddLoadedSearchPath(initInfo, "MOD", szAbsSearchPath, false);
    }
  }

  bool bLowViolence = initInfo.m_bLowViolence;
  for (KeyValues *pCur = pSearchPaths->GetFirstValue(); pCur;
       pCur = pCur->GetNextValue()) {
    const char *pszPathID = pCur->GetName();
    const char *pLocation = pCur->GetString();
    const char *pszBaseDir = baseDir;
    char szAppInstallDir[1024];

    if (!FileSystem_AllowedSearchPath(pLocation))
      continue;

    if (Q_stristr(pLocation, APPID_PREFIX_TOKEN) == pLocation) {
      pLocation += strlen(APPID_PREFIX_TOKEN);
      const char *pNumberLoc = pLocation;
      int nAppId = V_atoi(pNumberLoc);
      pLocation = Q_stristr(pLocation, "|");
      if (!pLocation) {
        Error("Malformed gameinfo.txt");
      }
      pLocation += strlen("|");

      if (!nAppId) {
        Error("Can't mount content from invalid appid.");
      }

      const Source1AppidInfo_t *pKnownAppid = GetKnownAppidInfo(nAppId);
      const char *pszAppName = pKnownAppid ? pKnownAppid->pszName : "Unknown";

#ifdef ENGINE_DLL
      if (!SteamApps()) {
        Error("No SteamApps connection.");
      }

      if (!SteamApps()->BIsSubscribedApp(nAppId)) {
        char szStoreCommand[4096];
        V_sprintf_safe(szStoreCommand, "steam://store/%d", nAppId);
        Plat_OpenURL(szStoreCommand);

        Error("This mod requires that you own %s (%d). Please purchase it, and "
              "install it to play this mod.",
              pszAppName, nAppId);
      }

      if (!SteamApps()->BIsAppInstalled(nAppId)) {
        char szInstallCommand[4096];
        V_sprintf_safe(szInstallCommand, "steam://install/%d", nAppId);
        Plat_OpenURL(szInstallCommand);

        Error("This mod requires %s (%d) to be installed. Please install it to "
              "play this mod.",
              pszAppName, nAppId);
      }

      uint32 unLength = SteamApps()->GetAppInstallDir(nAppId, szAppInstallDir,
                                                      sizeof(szAppInstallDir));
      if (!unLength) {
        Error("Couldn't get install dir for appid: %d", nAppId);
      }
      pszBaseDir = szAppInstallDir;
#else
      if (!GetAppInstallDirFromSteamVDF(nAppId, szAppInstallDir, sizeof(szAppInstallDir))) {
        Warning("The gameinfo.txt references %s AppId %d, that is not installed. Skipping mount.\n", pszAppName, nAppId);
        continue;
      }
      pszBaseDir = szAppInstallDir;
#endif
    } else if (Q_stristr(pLocation, GAMEINFOPATH_TOKEN) == pLocation) {
      pLocation += strlen(GAMEINFOPATH_TOKEN);
      pszBaseDir = initInfo.m_pDirectoryName;
    } else if (Q_stristr(pLocation, BASESOURCEPATHS_TOKEN) == pLocation) {
      // This is a special identifier that tells it to add the specified path
      // for all source engine versions equal to or prior to this version. So in
      // Orange Box, if they specified:
      //		|all_source_engine_paths|hl2
      // it would add the ep2\hl2 folder and the base (ep1-era) hl2 folder.
      //
      // We need a special identifier in the gameinfo.txt here because the base
      // hl2 folder exists in different places. In the case of a game or a
      // Steam-launched dedicated server, all the necessary prior engine content
      // is mapped in with the Steam depots, so we can just use the path as-is.
      pLocation += strlen(BASESOURCEPATHS_TOKEN);

      // Resolve relative to the -game directory's parent, not -binroot.
      // This allows using another game's gameinfo.txt (e.g. Garry's Mod)
      // with -binroot pointing at the SDK for compatible binaries only.
      const char *pGameDir = GetVProjectCmdLineValue();
      if (pGameDir) {
        static char gameParentDir[MAX_PATH];
        Q_MakeAbsolutePath(gameParentDir, sizeof(gameParentDir), pGameDir);
        Q_StripTrailingSlash(gameParentDir);
        Q_StripFilename(gameParentDir);
        pszBaseDir = gameParentDir;
      }
    }

    char szBinLocation[MAX_PATH];
    if (Q_stricmp(pszPathID, "GAMEBIN") == 0) {
      V_sprintf_safe(szBinLocation, "%s" PLATFORM_DIR, pLocation);
      pLocation = szBinLocation;
    }

    CUtlStringList vecFullLocationPaths;
    char szAbsSearchPath[MAX_PATH];
    V_MakeAbsolutePath(szAbsSearchPath, sizeof(szAbsSearchPath), pLocation,
                       pszBaseDir);

    // Now resolve any ./'s.
    V_FixSlashes(szAbsSearchPath);
    if (!V_RemoveDotSlashes(szAbsSearchPath))
      Error("FileSystem_AddLoadedSearchPath - Can't resolve pathname for '%s'",
            szAbsSearchPath);
    V_StripTrailingSlash(szAbsSearchPath);

    // Don't bother doing any wildcard expansion unless it has wildcards.  This
    // avoids the weird thing with xxx_dir.vpk files being referred to simply as
    // xxx.vpk.
    if (V_stristr(pLocation, "?") == NULL &&
        V_stristr(pLocation, "*") == NULL) {
      vecFullLocationPaths.CopyAndAddToTail(szAbsSearchPath);
    } else {
      FileFindHandle_t findHandle = NULL;
      const char *pszFoundShortName =
          initInfo.m_pFileSystem->FindFirst(szAbsSearchPath, &findHandle);
      if (pszFoundShortName) {
        do {

          // We only know how to mount VPK's and directories
          if (pszFoundShortName[0] != '.' &&
              (initInfo.m_pFileSystem->FindIsDirectory(findHandle) ||
               V_stristr(pszFoundShortName, ".vpk"))) {
            char szAbsName[MAX_PATH];
            V_ExtractFilePath(szAbsSearchPath, szAbsName, sizeof(szAbsName));
            V_AppendSlash(szAbsName, sizeof(szAbsName));
            V_strcat_safe(szAbsName, pszFoundShortName);

            vecFullLocationPaths.CopyAndAddToTail(szAbsName);

            // Check for a common mistake
            if (!V_stricmp(pszFoundShortName, "materials") ||
                !V_stricmp(pszFoundShortName, "maps") ||
                !V_stricmp(pszFoundShortName, "resource") ||
                !V_stricmp(pszFoundShortName, "scripts") ||
                !V_stricmp(pszFoundShortName, "sound") ||
                !V_stricmp(pszFoundShortName, "models")) {

              char szReadme[MAX_PATH];
              V_ExtractFilePath(szAbsSearchPath, szReadme, sizeof(szReadme));
              V_AppendSlash(szReadme, sizeof(szReadme));
              V_strcat_safe(szReadme, "readme.txt");

              Error("Tried to add %s as a search path.\n"
                    "\nThis is probably not what you intended.\n"
                    "\nCheck %s for more info\n",
                    szAbsName, szReadme);
            }
          }
          pszFoundShortName = initInfo.m_pFileSystem->FindNext(findHandle);
        } while (pszFoundShortName);
        initInfo.m_pFileSystem->FindClose(findHandle);
      }

      // Sort alphabetically.  Also note that this will put
      // all the xxx_000.vpk packs just before the corresponding
      // xxx_dir.vpk
      vecFullLocationPaths.Sort(SortStricmp);

      // Now for any _dir.vpk files, remove the _nnn.vpk ones.
      int idx = vecFullLocationPaths.Count() - 1;
      while (idx > 0) {
        char szTemp[MAX_PATH];
        V_strcpy_safe(szTemp, vecFullLocationPaths[idx]);
        --idx;

        char *szDirVpk = V_stristr(szTemp, "_dir.vpk");
        if (szDirVpk != NULL) {
          *szDirVpk = '\0';
          while (idx >= 0) {
            char *pszPath = vecFullLocationPaths[idx];
            if (V_stristr(pszPath, szTemp) != pszPath)
              break;
            delete pszPath;
            vecFullLocationPaths.Remove(idx);
            --idx;
          }
        }
      }
    }

    // Parse Path ID list
    CUtlStringList vecPathIDs;
    V_SplitString(pCur->GetName(), "+", vecPathIDs);
    FOR_EACH_VEC(vecPathIDs, idxPathID) {
      Q_StripPrecedingAndTrailingWhitespace(vecPathIDs[idxPathID]);
    }

    // Mount them.
    FOR_EACH_VEC(vecFullLocationPaths, idxLocation) {
      FOR_EACH_VEC(vecPathIDs, idxPathID) {
        FileSystem_AddLoadedSearchPath(initInfo, vecPathIDs[idxPathID],
                                       vecFullLocationPaths[idxLocation],
                                       bLowViolence);
      }
    }
  }

  pMainFile->deleteThis();

  // Also, mark specific path IDs as "by request only". That way, we won't waste
  // time searching in them when people forget to specify a search path.
  initInfo.m_pFileSystem->MarkPathIDByRequestOnly("executable_path", true);
  initInfo.m_pFileSystem->MarkPathIDByRequestOnly("gamebin", true);
  initInfo.m_pFileSystem->MarkPathIDByRequestOnly("download", true);
  initInfo.m_pFileSystem->MarkPathIDByRequestOnly("mod", true);
  initInfo.m_pFileSystem->MarkPathIDByRequestOnly("game_write", true);
  initInfo.m_pFileSystem->MarkPathIDByRequestOnly("mod_write", true);

  return FS_OK;
}

bool DoesFileExistIn(const char *pDirectoryName, const char *pFilename) {
  char filename[MAX_PATH];

  Q_strncpy(filename, pDirectoryName, sizeof(filename));
  Q_AppendSlash(filename, sizeof(filename));
  Q_strncat(filename, pFilename, sizeof(filename), COPY_ALL_CHARACTERS);
  Q_FixSlashes(filename);
  bool bExist = (_access(filename, 0) == 0);

  return (bExist);
}

namespace {
SuggestGameInfoDirFn_t &GetSuggestGameInfoDirFn(void) {
  static SuggestGameInfoDirFn_t s_pfnSuggestGameInfoDir = NULL;
  return s_pfnSuggestGameInfoDir;
}
}; // namespace

SuggestGameInfoDirFn_t
SetSuggestGameInfoDirFn(SuggestGameInfoDirFn_t pfnNewFn) {
  SuggestGameInfoDirFn_t &rfn = GetSuggestGameInfoDirFn();
  SuggestGameInfoDirFn_t pfnOldFn = rfn;
  rfn = pfnNewFn;
  return pfnOldFn;
}

static FSReturnCode_t TryLocateGameInfoFile(char *pOutDir, int outDirLen,
                                            bool bBubbleDir) {
  // Retain a copy of suggested path for further attempts
  CArrayAutoPtr<char> spchCopyNameBuffer(new char[outDirLen]);
  Q_strncpy(spchCopyNameBuffer.Get(), pOutDir, outDirLen);
  spchCopyNameBuffer[outDirLen - 1] = 0;

  // Make appropriate slashes ('/' - Linux style)
  for (char *pchFix = spchCopyNameBuffer.Get(), *pchEnd = pchFix + outDirLen;
       pchFix < pchEnd; ++pchFix) {
    if ('\\' == *pchFix) {
      *pchFix = '/';
    }
  }

  // Have a look in supplied path
  do {
    if (DoesFileExistIn(pOutDir, GAMEINFO_FILENAME)) {
      return FS_OK;
    }
    if (IsX360() && DoesFileExistIn(pOutDir, GAMEINFO_FILENAME_ALTERNATE)) {
      return FS_OK;
    }
  } while (bBubbleDir && Q_StripLastDir(pOutDir, outDirLen));

  // Make an attempt to resolve from "content -> game" directory
  Q_strncpy(pOutDir, spchCopyNameBuffer.Get(), outDirLen);
  pOutDir[outDirLen - 1] = 0;
  if (char *pchContentFix = Q_stristr(pOutDir, "/content/")) {
    sprintf(pchContentFix, "/game/");
    memmove(pchContentFix + 6, pchContentFix + 9,
            pOutDir + outDirLen - (pchContentFix + 9));

    // Try in the mapped "game" directory
    do {
      if (DoesFileExistIn(pOutDir, GAMEINFO_FILENAME)) {
        return FS_OK;
      }
      if (IsX360() && DoesFileExistIn(pOutDir, GAMEINFO_FILENAME_ALTERNATE)) {
        return FS_OK;
      }
    } while (bBubbleDir && Q_StripLastDir(pOutDir, outDirLen));
  }

  // Could not find it here
  return FS_MISSING_GAMEINFO_FILE;
}

FSReturnCode_t LocateGameInfoFile(const CFSSteamSetupInfo &fsInfo,
                                  char *pOutDir, int outDirLen) {
  // Engine and Hammer don't want to search around for it.
  if (fsInfo.m_bOnlyUseDirectoryName) {
    if (!fsInfo.m_pDirectoryName)
      return SetupFileSystemError(
          false, FS_MISSING_GAMEINFO_FILE,
          "bOnlyUseDirectoryName=1 and pDirectoryName=NULL.");

    bool bExists = DoesFileExistIn(fsInfo.m_pDirectoryName, GAMEINFO_FILENAME);
    if (IsX360() && !bExists) {
      bExists =
          DoesFileExistIn(fsInfo.m_pDirectoryName, GAMEINFO_FILENAME_ALTERNATE);
    }
    if (!bExists) {
      if (IsX360() && CommandLine()->FindParm("-basedir")) {
        char basePath[MAX_PATH];
        strcpy(basePath, CommandLine()->ParmValue("-basedir", ""));
        Q_AppendSlash(basePath, sizeof(basePath));
        Q_strncat(basePath, fsInfo.m_pDirectoryName, sizeof(basePath),
                  COPY_ALL_CHARACTERS);
        if (DoesFileExistIn(basePath, GAMEINFO_FILENAME)) {
          Q_strncpy(pOutDir, basePath, outDirLen);
          return FS_OK;
        }
        if (IsX360() &&
            DoesFileExistIn(basePath, GAMEINFO_FILENAME_ALTERNATE)) {
          Q_strncpy(pOutDir, basePath, outDirLen);
          return FS_OK;
        }
      }

      return SetupFileSystemError(
          true, FS_MISSING_GAMEINFO_FILE,
          "Setup file '%s' doesn't exist in subdirectory '%s'.\nCheck your "
          "-game parameter or VCONFIG setting.",
          GAMEINFO_FILENAME, fsInfo.m_pDirectoryName);
    }

    Q_strncpy(pOutDir, fsInfo.m_pDirectoryName, outDirLen);
    return FS_OK;
  }

  // First, check for overrides on the command line or environment variables.
  const char *pProject = GetVProjectCmdLineValue();

  if (pProject) {
    if (DoesFileExistIn(pProject, GAMEINFO_FILENAME)) {
      Q_MakeAbsolutePath(pOutDir, outDirLen, pProject);
      return FS_OK;
    }
    if (IsX360() && DoesFileExistIn(pProject, GAMEINFO_FILENAME_ALTERNATE)) {
      Q_MakeAbsolutePath(pOutDir, outDirLen, pProject);
      return FS_OK;
    }

    if (IsX360() && CommandLine()->FindParm("-basedir")) {
      char basePath[MAX_PATH];
      strcpy(basePath, CommandLine()->ParmValue("-basedir", ""));
      Q_AppendSlash(basePath, sizeof(basePath));
      Q_strncat(basePath, pProject, sizeof(basePath), COPY_ALL_CHARACTERS);
      if (DoesFileExistIn(basePath, GAMEINFO_FILENAME)) {
        Q_strncpy(pOutDir, basePath, outDirLen);
        return FS_OK;
      }
      if (DoesFileExistIn(basePath, GAMEINFO_FILENAME_ALTERNATE)) {
        Q_strncpy(pOutDir, basePath, outDirLen);
        return FS_OK;
      }
    }

    if (fsInfo.m_bNoGameInfo) {
      // fsInfo.m_bNoGameInfo is set by the Steam dedicated server, before it
      // knows which mod to use. Steam dedicated server doesn't need a
      // gameinfo.txt, because we'll ask which mod to use, even if -game is
      // supplied on the command line.
      Q_strncpy(pOutDir, "", outDirLen);
      return FS_OK;
    } else {
      // They either specified vproject on the command line or it's in their
      // registry. Either way, we don't want to continue if they've specified it
      // but it's not valid.
      goto ShowError;
    }
  }

  if (fsInfo.m_bNoGameInfo) {
    Q_strncpy(pOutDir, "", outDirLen);
    return FS_OK;
  }

  // Ask the application if it can provide us with a game info directory
  {
    bool bBubbleDir = true;
    SuggestGameInfoDirFn_t pfnSuggestGameInfoDirFn = GetSuggestGameInfoDirFn();
    if (pfnSuggestGameInfoDirFn &&
        (*pfnSuggestGameInfoDirFn)(&fsInfo, pOutDir, outDirLen, &bBubbleDir) &&
        FS_OK == TryLocateGameInfoFile(pOutDir, outDirLen, bBubbleDir))
      return FS_OK;
  }

  // Try to use the environment variable / registry
  if ((pProject = getenv(GAMEDIR_TOKEN)) != NULL &&
      (Q_MakeAbsolutePath(pOutDir, outDirLen, pProject), 1) &&
      FS_OK == TryLocateGameInfoFile(pOutDir, outDirLen, false))
    return FS_OK;

  if (IsPC()) {
    Warning("Warning: falling back to auto detection of vproject directory.\n");

    // Now look for it in the directory they passed in.
    if (fsInfo.m_pDirectoryName)
      Q_MakeAbsolutePath(pOutDir, outDirLen, fsInfo.m_pDirectoryName);
    else
      Q_MakeAbsolutePath(pOutDir, outDirLen, ".");

    if (FS_OK == TryLocateGameInfoFile(pOutDir, outDirLen, true))
      return FS_OK;

    // Use the CWD
    Q_getwd(pOutDir, outDirLen);
    if (FS_OK == TryLocateGameInfoFile(pOutDir, outDirLen, true))
      return FS_OK;
  }

ShowError:
  return SetupFileSystemError(
      true, FS_MISSING_GAMEINFO_FILE,
      "Unable to find %s. Solutions:\n\n"
      "1. Read https://developer.valvesoftware.com/wiki/Gameinfo.txt\n"
      "2. Run vconfig to specify which game you're working on.\n"
      "3. Add -game <path> on the command line where <path> is the directory "
      "that %s is in.\n",
      GAMEINFO_FILENAME, GAMEINFO_FILENAME);
}

bool DoesPathExistAlready(const char *pPathEnvVar, const char *pTestPath) {
  // Fix the slashes in the input arguments.
  char correctedPathEnvVar[8192], correctedTestPath[MAX_PATH];
  Q_strncpy(correctedPathEnvVar, pPathEnvVar, sizeof(correctedPathEnvVar));
  Q_FixSlashes(correctedPathEnvVar);
  pPathEnvVar = correctedPathEnvVar;

  Q_strncpy(correctedTestPath, pTestPath, sizeof(correctedTestPath));
  Q_FixSlashes(correctedTestPath);
  if (strlen(correctedTestPath) > 0 &&
      PATHSEPARATOR(correctedTestPath[strlen(correctedTestPath) - 1]))
    correctedTestPath[strlen(correctedTestPath) - 1] = 0;

  pTestPath = correctedTestPath;

  const char *pCurPos = pPathEnvVar;
  while (1) {
    const char *pTestPos = Q_stristr(pCurPos, pTestPath);
    if (!pTestPos)
      return false;

    // Ok, we found pTestPath in the path, but it's only valid if it's followed
    // by an optional slash and a semicolon.
    pTestPos += strlen(pTestPath);
    if (pTestPos[0] == 0 || pTestPos[0] == ';' ||
        (PATHSEPARATOR(pTestPos[0]) && pTestPos[1] == ';'))
      return true;

    // Advance our marker..
    pCurPos = pTestPos;
  }
}

FSReturnCode_t GetSteamCfgPath(char *steamCfgPath, int steamCfgPathLen) {
  steamCfgPath[0] = 0;
  char executablePath[MAX_PATH];
  if (!FileSystem_GetExecutableDir(executablePath, sizeof(executablePath))) {
    return SetupFileSystemError(false, FS_INVALID_PARAMETERS,
                                "FileSystem_GetExecutableDir failed.");
  }
  Q_strncpy(steamCfgPath, executablePath, steamCfgPathLen);
  while (1) {
    if (DoesFileExistIn(steamCfgPath, "steam.cfg"))
      break;

    if (!Q_StripLastDir(steamCfgPath, steamCfgPathLen)) {
      // the file isnt found, thats ok, its not mandatory
      return FS_OK;
    }
  }
  Q_AppendSlash(steamCfgPath, steamCfgPathLen);
  Q_strncat(steamCfgPath, "steam.cfg", steamCfgPathLen, COPY_ALL_CHARACTERS);

  return FS_OK;
}

void SetSteamAppUser(KeyValues *pSteamInfo, const char *steamInstallPath,
                     CSteamEnvVars &steamEnvVars) {
  // Always inherit the Steam user if it's already set, since it probably means
  // we (or the the app that launched us) were launched from Steam.
  char appUser[MAX_PATH];
  if (steamEnvVars.m_SteamAppUser.GetValue(appUser, sizeof(appUser)))
    return;

  const char *pTempAppUser = NULL;
  if (pSteamInfo &&
      (pTempAppUser = pSteamInfo->GetString("SteamAppUser", NULL)) != NULL) {
    Q_strncpy(appUser, pTempAppUser, sizeof(appUser));
  } else {
    // They don't have SteamInfo.txt, or it's missing SteamAppUser. Try to
    // figure out the user by looking in <steam install
    // path>\config\SteamAppData.vdf.
    char fullFilename[MAX_PATH];
    Q_strncpy(fullFilename, steamInstallPath, sizeof(fullFilename));
    Q_AppendSlash(fullFilename, sizeof(fullFilename));
    Q_strncat(fullFilename, "config\\SteamAppData.vdf", sizeof(fullFilename),
              COPY_ALL_CHARACTERS);

    KeyValues *pSteamAppData = ReadKeyValuesFile(fullFilename);
    if (!pSteamAppData || (pTempAppUser = pSteamAppData->GetString(
                               "AutoLoginUser", NULL)) == NULL) {
      Error("Can't find steam app user info.");
    }
    Q_strncpy(appUser, pTempAppUser, sizeof(appUser));

    pSteamAppData->deleteThis();
  }

  Q_strlower(appUser);
  steamEnvVars.m_SteamAppUser.SetValue("%s", appUser);
}

void SetSteamUserPassphrase(KeyValues *pSteamInfo,
                            CSteamEnvVars &steamEnvVars) {
  // Always inherit the passphrase if it's already set, since it probably means
  // we (or the the app that launched us) were launched from Steam.
  char szPassPhrase[MAX_PATH];
  if (steamEnvVars.m_SteamUserPassphrase.GetValue(szPassPhrase,
                                                  sizeof(szPassPhrase)))
    return;

  // SteamUserPassphrase.
  const char *pStr;
  if (pSteamInfo &&
      (pStr = pSteamInfo->GetString("SteamUserPassphrase", NULL)) != NULL) {
    steamEnvVars.m_SteamUserPassphrase.SetValue("%s", pStr);
  }
}

FSReturnCode_t FileSystem_SetBasePaths(IFileSystem *pFileSystem) {
  pFileSystem->RemoveSearchPaths("EXECUTABLE_PATH");

  char executablePath[MAX_PATH];
  if (!FileSystem_GetExecutableDir(executablePath, sizeof(executablePath)))
    return SetupFileSystemError(false, FS_INVALID_PARAMETERS,
                                "FileSystem_GetExecutableDir failed.");

  pFileSystem->AddSearchPath(executablePath, "EXECUTABLE_PATH");
  if (*PLATFORM_DIR) {
    char baseBinFolder[MAX_PATH];
    Q_strncpy(baseBinFolder, executablePath, MAX_PATH);
    baseBinFolder[V_strlen(baseBinFolder) - V_strlen(PLATFORM_DIR)] = '\0';
    pFileSystem->AddSearchPath(baseBinFolder, "EXECUTABLE_PATH");
  }

  if (!FileSystem_GetBaseDir(executablePath, sizeof(executablePath)))
    return SetupFileSystemError(false, FS_INVALID_PARAMETERS,
                                "FileSystem_GetBaseDir failed.");

  pFileSystem->AddSearchPath(executablePath, "BASE_PATH");

  return FS_OK;
}

//-----------------------------------------------------------------------------
// Returns the name of the file system DLL to use
//-----------------------------------------------------------------------------
FSReturnCode_t FileSystem_GetFileSystemDLLName(char *pFileSystemDLL,
                                               int nMaxLen, bool &bSteam) {
  bSteam = false;

  // Always load the filesystem DLL from our own executable's directory.
  // This is intentionally separate from FileSystem_GetExecutableDir(), which
  // may return a content root's bin dir for EXECUTABLE_PATH resolution.
  // We must use our own bundled filesystem_stdio.dll because other games
  // (e.g. Garry's Mod) may ship incompatible versions.
  char executablePath[MAX_PATH];
  if (!FileSystem_GetRealExecutableDir(executablePath, sizeof(executablePath)))
    return SetupFileSystemError(false, FS_INVALID_PARAMETERS,
                                "FileSystem_GetRealExecutableDir failed.");

  // Assume we'll use local files
  Q_snprintf(pFileSystemDLL, nMaxLen, "%s%cfilesystem_stdio" DLL_EXT_STRING,
             executablePath, CORRECT_PATH_SEPARATOR);

#if !defined(_X360)

// Use filesystem_steam if it exists?
#if defined(OSX) || defined(LINUX)
  struct stat statBuf;
#endif
  if (
#if defined(OSX) || defined(LINUX)
      stat(pFileSystemDLL, &statBuf) != 0
#else
      _access(pFileSystemDLL, 0) != 0
#endif
  ) {
    Q_snprintf(pFileSystemDLL, nMaxLen, "%s%cfilesystem_steam" DLL_EXT_STRING,
               executablePath, CORRECT_PATH_SEPARATOR);
    bSteam = true;
  }
#endif

  return FS_OK;
}

//-----------------------------------------------------------------------------
// Sets up the steam environment + gets back the gameinfo.txt path
//-----------------------------------------------------------------------------
FSReturnCode_t FileSystem_SetupSteamEnvironment(CFSSteamSetupInfo &fsInfo) {
  // First, locate the directory with gameinfo.txt.
  FSReturnCode_t ret = LocateGameInfoFile(fsInfo, fsInfo.m_GameInfoPath,
                                          sizeof(fsInfo.m_GameInfoPath));
  if (ret != FS_OK)
    return ret;

  // This is so that processes spawned by this application will have the same
  // VPROJECT
#ifdef WIN32
  char pEnvBuf[MAX_PATH + 32];
  Q_snprintf(pEnvBuf, sizeof(pEnvBuf), "%s=%s", GAMEDIR_TOKEN,
             fsInfo.m_GameInfoPath);
  _putenv(pEnvBuf);
#else
  setenv(GAMEDIR_TOKEN, fsInfo.m_GameInfoPath, 1);
#endif

  return FS_OK;
}

//-----------------------------------------------------------------------------
// Loads the file system module
//-----------------------------------------------------------------------------
FSReturnCode_t FileSystem_LoadFileSystemModule(CFSLoadModuleInfo &fsInfo) {
  // First, locate the directory with gameinfo.txt.
  FSReturnCode_t ret = FileSystem_SetupSteamEnvironment(fsInfo);
  if (ret != FS_OK)
    return ret;

  // Now that the environment is setup, load the filesystem module.
  if (!Sys_LoadInterface(fsInfo.m_pFileSystemDLLName,
                         FILESYSTEM_INTERFACE_VERSION, &fsInfo.m_pModule,
                         (void **)&fsInfo.m_pFileSystem)) {
    return SetupFileSystemError(false, FS_UNABLE_TO_INIT, "Can't load %s.",
                                fsInfo.m_pFileSystemDLLName);
  }

  if (!fsInfo.m_pFileSystem->Connect(fsInfo.m_ConnectFactory))
    return SetupFileSystemError(false, FS_UNABLE_TO_INIT,
                                "%s IFileSystem::Connect failed.",
                                fsInfo.m_pFileSystemDLLName);

  if (fsInfo.m_pFileSystem->Init() != INIT_OK)
    return SetupFileSystemError(false, FS_UNABLE_TO_INIT,
                                "%s IFileSystem::Init failed.",
                                fsInfo.m_pFileSystemDLLName);

  return FS_OK;
}

//-----------------------------------------------------------------------------
// Mounds a particular steam cache
//-----------------------------------------------------------------------------
FSReturnCode_t FileSystem_MountContent(CFSMountContentInfo &mountContentInfo) {
  // This part is Steam-only.
  if (mountContentInfo.m_pFileSystem->IsSteam()) {
    return SetupFileSystemError(
        false, FS_INVALID_PARAMETERS,
        "Should not be using filesystem_steam anymore!");

    //		// Find out the "extra app id". This is for tools, which want to
    // mount a base app's filesystem
    //		// like HL2, then mount the SDK content (tools materials and
    // models, etc) in addition. 		int nExtraAppId = -1; if (
    // mountContentInfo.m_bToolsMode )
    //		{
    //			// !FIXME! Here we need to mount the tools content
    //(VPK's) in some way...?
    //		}
    //
    //		// Set our working directory temporarily so Steam can remember
    // it.
    //		// This is what Steam strips off absolute filenames like
    // c:\program files\valve\steam\steamapps\username\sourcesdk
    //		// to get to the relative part of the path.
    //		char baseDir[MAX_PATH], oldWorkingDir[MAX_PATH];
    //		if ( !FileSystem_GetBaseDir( baseDir, sizeof( baseDir ) ) )
    //			return SetupFileSystemError( false,
    // FS_INVALID_PARAMETERS, "FileSystem_GetBaseDir failed." );
    //
    //		Q_getwd( oldWorkingDir, sizeof( oldWorkingDir ) );
    //		_chdir( baseDir );
    //
    //		// Filesystem_tools needs to add dependencies in here
    // beforehand. 		FilesystemMountRetval_t retVal =
    // mountContentInfo.m_pFileSystem->MountSteamContent( nExtraAppId );
    //
    //		_chdir( oldWorkingDir );
    //
    //		if ( retVal != FILESYSTEM_MOUNT_OK )
    //			return SetupFileSystemError( true, FS_UNABLE_TO_INIT,
    //"Unable to mount Steam content in the file system" );
  }

  return FileSystem_SetBasePaths(mountContentInfo.m_pFileSystem);
}

void FileSystem_SetErrorMode(FSErrorMode_t errorMode) {
  g_FileSystemErrorMode = errorMode;
}

void FileSystem_ClearSteamEnvVars() {
  CSteamEnvVars envVars;

  // Change the values and don't restore the originals.
  envVars.m_SteamAppId.SetValue("");
  envVars.m_SteamUserPassphrase.SetValue("");
  envVars.m_SteamAppUser.SetValue("");

  envVars.SetRestoreOriginalValue_ALL(false);
}

//-----------------------------------------------------------------------------
// Adds the platform folder to the search path.
//-----------------------------------------------------------------------------
void FileSystem_AddSearchPath_Platform(IFileSystem *pFileSystem,
                                       const char *szGameInfoPath) {
  char platform[MAX_PATH];
  Q_strncpy(platform, szGameInfoPath, MAX_PATH);
  Q_StripTrailingSlash(platform);
  Q_strncat(platform, "/../platform", MAX_PATH, MAX_PATH);

  pFileSystem->AddSearchPath(platform, "PLATFORM");
}

bool FileSystem_AllowedSearchPath(const char *pchPath) {
  if (V_stristr(pchPath, "WebDavRedirector")) {
    Assert(!"Removing possibly malicious search path!");
    return false;
  }

  return true;
}
