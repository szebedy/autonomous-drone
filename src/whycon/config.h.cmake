#ifndef __CONFIG_H__
#define __CONFIG_H__

#cmakedefine ENABLE_FULL_UNDISTORT
#cmakedefine ENABLE_RANDOMIZED_THRESHOLD
#cmakedefine ENABLE_VERBOSE

#if defined(ENABLE_VERBOSE)
#define WHYCON_DEBUG(x) cout << x << endl
#else
#define WHYCON_DEBUG(x)
#endif

#endif
