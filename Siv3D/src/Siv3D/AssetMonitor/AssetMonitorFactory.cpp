﻿//-----------------------------------------------
//
//	This file is part of the Siv3D Engine.
//
//	Copyright (c) 2008-2022 Ryo Suzuki
//	Copyright (c) 2016-2022 OpenSiv3D Project
//
//	Licensed under the MIT License.
//
//-----------------------------------------------

# include "CAssetMonitor.hpp"

namespace s3d
{
	ISiv3DAssetMonitor* ISiv3DAssetMonitor::Create()
	{
		return new CAssetMonitor;
	}
}
