//-----------------------------------------------
//
//	This file is part of the Siv3D Engine.
//
//	Copyright (c) 2008-2025 Ryo Suzuki
//	Copyright (c) 2016-2025 OpenSiv3D Project
//
//	Licensed under the MIT License.
//
//-----------------------------------------------

# pragma once
# include "Platform.hpp"

# if SIV3D_INTRINSIC(SSE)

# if __has_include(<immintrin.h>)
#	include <immintrin.h>
# endif

# else

# define SIMDE_ENABLE_NATIVE_ALIASES

# include <ThirdParty/simde/x86/sse.h>
# include <ThirdParty/simde/x86/sse2.h>
# include <ThirdParty/simde/x86/sse3.h>
# include <ThirdParty/simde/x86/ssse3.h>
# include <ThirdParty/simde/x86/sse4.1.h>
# include <ThirdParty/simde/x86/sse4.2.h>

# include <malloc.h>    // For memalign

# define _mm_malloc(__size, __align) memalign((__align), (__size))
# define _mm_free free

# endif
