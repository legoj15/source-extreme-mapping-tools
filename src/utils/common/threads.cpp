//========= Copyright Valve Corporation, All rights reserved. ============//
//
// Purpose:
//
// $Workfile:     $
// $Date:         $
//
//-----------------------------------------------------------------------------
// $Log: $
//
// $NoKeywords: $
//=============================================================================//

#define USED

#include "cmdlib.h"
#include <windows.h>

#define NO_THREAD_NAMES
#include "pacifier.h"
#include "threads.h"


#define MAX_THREADS MAX_TOOL_THREADS

class CRunThreadsData {
public:
  int m_iThread;
  void *m_pUserData;
  RunThreadsFn m_Fn;
};

CRunThreadsData g_RunThreadsData[MAX_THREADS];

int dispatch;
int workcount;
qboolean pacifier;

// Serializes pacifier drawing between worker threads. Separate from the
// dispatch critical section so a thread blocked writing to a full stdout
// pipe can never stall work dispatch (the "Lock Trap" deadlock).
CRITICAL_SECTION g_PacifierCrit;

qboolean threaded;
bool g_bLowPriorityThreads = false;

HANDLE g_ThreadHandles[MAX_THREADS];

/*
=============
GetThreadWork

=============
*/
int GetThreadWork(void) {
  int r;
  int total;

  ThreadLock();

  if (dispatch == workcount) {
    ThreadUnlock();
    return -1;
  }

  r = dispatch;
  total = workcount;
  dispatch++;
  ThreadUnlock();

  // Draw the pacifier outside the dispatch lock: Msg() blocks indefinitely
  // when stdout is a pipe whose buffer is full, and holding the dispatch
  // critical section across that write deadlocks every worker. TryEnter so
  // workers never queue behind a blocked writer; a skipped update is redrawn
  // by the next successful one since the pacifier only moves forward.
  if (TryEnterCriticalSection(&g_PacifierCrit)) {
    UpdatePacifier((float)r / total);
    LeaveCriticalSection(&g_PacifierCrit);
  }

  return r;
}

ThreadWorkerFn workfunction;

void ThreadWorkerFunction(int iThread, void *pUserData) {
  int work;

  while (1) {
    work = GetThreadWork();
    if (work == -1)
      break;

    workfunction(iThread, work);
  }
}

void RunThreadsOnIndividual(int workcnt, qboolean showpacifier,
                            ThreadWorkerFn func) {
  if (numthreads == -1)
    ThreadSetDefault();

  workfunction = func;
  RunThreadsOn(workcnt, showpacifier, ThreadWorkerFunction);
}

/*
===================================================================

WIN32

===================================================================
*/

int numthreads = -1;
CRITICAL_SECTION crit;
static int enter;

class CCritInit {
public:
  CCritInit() {
    InitializeCriticalSection(&crit);
    InitializeCriticalSection(&g_PacifierCrit);
  }
} g_CritInit;

void SetLowPriority() {
  SetPriorityClass(GetCurrentProcess(), IDLE_PRIORITY_CLASS);
}

void ThreadSetDefault(void) {
  SYSTEM_INFO info;

  if (numthreads == -1) // not set manually
  {
    GetSystemInfo(&info);
    numthreads = info.dwNumberOfProcessors;
    if (numthreads < 1 || numthreads > 64)
      numthreads = 1;
  }

  Msg("%i threads\n", numthreads);
}

void ThreadLock(void) {
  if (!threaded)
    return;
  EnterCriticalSection(&crit);
  if (enter)
    Error("Recursive ThreadLock\n");
  enter = 1;
}

void ThreadUnlock(void) {
  if (!threaded)
    return;
  if (!enter)
    Error("ThreadUnlock without lock\n");
  enter = 0;
  LeaveCriticalSection(&crit);
}

// This runs in the thread and dispatches a RunThreadsFn call.
DWORD WINAPI InternalRunThreadsFn(LPVOID pParameter) {
  CRunThreadsData *pData = (CRunThreadsData *)pParameter;
  pData->m_Fn(pData->m_iThread, pData->m_pUserData);
  return 0;
}

void RunThreads_Start(RunThreadsFn fn, void *pUserData,
                      ERunThreadsPriority ePriority) {
  Assert(numthreads > 0);
  threaded = true;

  if (numthreads > MAX_TOOL_THREADS)
    numthreads = MAX_TOOL_THREADS;

  for (int i = 0; i < numthreads; i++) {
    g_RunThreadsData[i].m_iThread = i;
    g_RunThreadsData[i].m_pUserData = pUserData;
    g_RunThreadsData[i].m_Fn = fn;

    DWORD dwDummy;
    g_ThreadHandles[i] = CreateThread(
        NULL,                 // LPSECURITY_ATTRIBUTES lpsa,
        0,                    // DWORD cbStack,
        InternalRunThreadsFn, // LPTHREAD_START_ROUTINE lpStartAddr,
        &g_RunThreadsData[i], // LPVOID lpvThreadParm,
        0,                    // DWORD fdwCreate,
        &dwDummy);

    if (ePriority == k_eRunThreadsPriority_UseGlobalState) {
      if (g_bLowPriorityThreads)
        SetThreadPriority(g_ThreadHandles[i], THREAD_PRIORITY_LOWEST);
    } else if (ePriority == k_eRunThreadsPriority_Idle) {
      SetThreadPriority(g_ThreadHandles[i], THREAD_PRIORITY_IDLE);
    }
  }
}

void RunThreads_End() {
  WaitForMultipleObjects(numthreads, g_ThreadHandles, TRUE, INFINITE);
  for (int i = 0; i < numthreads; i++)
    CloseHandle(g_ThreadHandles[i]);

  threaded = false;
}

/*
=============
RunThreadsOn
=============
*/
void RunThreadsOn(int workcnt, qboolean showpacifier, RunThreadsFn fn,
                  void *pUserData) {
  int start, end;

  start = Plat_FloatTime();
  dispatch = 0;
  workcount = workcnt;
  StartPacifier("");
  pacifier = showpacifier;

#ifdef _PROFILE
  threaded = false;
  (*func)(0);
  return;
#endif

  RunThreads_Start(fn, pUserData);
  RunThreads_End();

  end = Plat_FloatTime();
  if (pacifier) {
    EndPacifier(false);
    printf(" (%i)\n", end - start);
  }
}
