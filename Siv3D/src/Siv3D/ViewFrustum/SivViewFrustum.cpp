//-----------------------------------------------
//
//	This file is part of the Siv3D Engine.
//
//	Copyright (c) 2008-2021 Ryo Suzuki
//	Copyright (c) 2016-2021 OpenSiv3D Project
//
//	Licensed under the MIT License.
//
//-----------------------------------------------

# include <Siv3D/ViewFrustum.hpp>
# include <Siv3D/SIMD_Float4.hpp>
# include <Siv3D/Quaternion.hpp>
# include <Siv3D/Spherical.hpp>
# include <Siv3D/Line3D.hpp>
# include <Siv3D/Geometry3D.hpp>

namespace s3d
{
	namespace detail
	{
		constexpr std::array<size_t, 8> CornerIndices =
		{
			0, 1, 3, 2, 4, 5, 7, 6
		};
	}

	ViewFrustum::ViewFrustum(const BasicCamera3D& camera, const double farClip) noexcept
		: ViewFrustum{ camera.getSceneSize(), camera.getVerticlaFOV(), camera.getEyePosition(), camera.getFocusPosition(), camera.getUpDirection(), camera.getNearClip(), farClip } {}

	ViewFrustum::ViewFrustum(const BasicCamera3D& camera, const double nearClip, const double farClip) noexcept
		: ViewFrustum{ camera.getSceneSize(), camera.getVerticlaFOV(), camera.getEyePosition(), camera.getFocusPosition(), camera.getUpDirection(), nearClip, farClip } {}

	ViewFrustum::ViewFrustum(const Size& sceneSize, const double verticalFOV, const Vec3& eyePosition, const Vec3& focusPosition, const Vec3& upDirection, const double nearClip, const double farClip) noexcept
	{
		const auto proj = DirectX::XMMatrixPerspectiveFovLH(
			static_cast<float>(verticalFOV),
			(static_cast<float>(sceneSize.x) / sceneSize.y),
			static_cast<float>(nearClip),
			static_cast<float>(farClip));

		DirectX::BoundingFrustum::CreateFromMatrix(m_frustum, proj);
		m_frustum.Origin = DirectX::XMFLOAT3{ static_cast<float>(eyePosition.x), static_cast<float>(eyePosition.y), static_cast<float>(eyePosition.z) };

		const Quaternion q = Quaternion::FromUnitVectorPairs(
			{ Vec3::Forward(), Vec3::Up() }, { (focusPosition - eyePosition).normalized(), upDirection });

		const Float4 o = q.toFloat4();

		m_frustum.Orientation = { o.x, o.y, o.z, o.w };
	}

	std::array<Vec3, 8> ViewFrustum::getCorners() const noexcept
	{
		std::array<DirectX::XMFLOAT3, 8> corners;

		m_frustum.GetCorners(corners.data());

		std::array<Vec3, 8> results;

		for (size_t i = 0; i < 8; ++i)
		{
			const auto& corner = corners[detail::CornerIndices[i]];

			results[i].set(corner.x, corner.y, corner.z);
		}

		return results;
	}

	bool ViewFrustum::intersects(const Vec3& point) const noexcept
	{
		const auto result = m_frustum.Contains(SIMD_Float4{ point, 0.0f });

		return (result == DirectX::ContainmentType::CONTAINS);
	}

	bool ViewFrustum::intersects(const Triangle3D& triangle) const noexcept
	{
		return m_frustum.Intersects(triangle.p0, triangle.p1, triangle.p2);
	}

	bool ViewFrustum::intersects(const Sphere& sphere) const noexcept
	{
		return m_frustum.Intersects(detail::FromSphere(sphere));
	}

	bool ViewFrustum::intersects(const Box& box) const noexcept
	{
		return m_frustum.Intersects(detail::FromBox(box));
	}

	bool ViewFrustum::intersects(const OrientedBox& box) const noexcept
	{
		return m_frustum.Intersects(detail::FromOrientedBox(box));
	}

	//-------------------------------------------------------------------------------------
	// DirectXCollision.inl -- C++ Collision Math library
	//
	// Copyright (c) Microsoft Corporation. All rights reserved.
	// Licensed under the MIT License.
	//
	// http://go.microsoft.com/fwlink/?LinkID=615560
	//-------------------------------------------------------------------------------------

	/*
	bool ViewFrustum::intersects(const ViewFrustum& frustum) const noexcept
	{
		using namespace DirectX;
		const auto& fr = frustum.getData();

		// Load origin and orientation of frustum B.
		XMVECTOR OriginB = XMLoadFloat3(&m_frustum.Origin);
		XMVECTOR OrientationB = XMLoadFloat4(&m_frustum.Orientation);

		assert(DirectX::Internal::XMQuaternionIsUnit(OrientationB));

		// Build the planes of frustum B.
		XMVECTOR AxisB[6];
		AxisB[0] = XMVectorSet(0.0f, 0.0f, -1.0f, 0.0f);
		AxisB[1] = XMVectorSet(0.0f, 0.0f, 1.0f, 0.0f);
		AxisB[2] = XMVectorSet(1.0f, 0.0f, -m_frustum.RightSlope, 0.0f);
		AxisB[3] = XMVectorSet(-1.0f, 0.0f, m_frustum.LeftSlope, 0.0f);
		AxisB[4] = XMVectorSet(0.0f, 1.0f, -m_frustum.TopSlope, 0.0f);
		AxisB[5] = XMVectorSet(0.0f, -1.0f, m_frustum.BottomSlope, 0.0f);

		XMVECTOR PlaneDistB[6];
		PlaneDistB[0] = XMVectorNegate(XMVectorReplicatePtr(&m_frustum.Near));
		PlaneDistB[1] = XMVectorReplicatePtr(&m_frustum.Far);
		PlaneDistB[2] = XMVectorZero();
		PlaneDistB[3] = XMVectorZero();
		PlaneDistB[4] = XMVectorZero();
		PlaneDistB[5] = XMVectorZero();

		// Load origin and orientation of frustum A.
		XMVECTOR OriginA = XMLoadFloat3(&fr.Origin);
		XMVECTOR OrientationA = XMLoadFloat4(&fr.Orientation);

		assert(DirectX::Internal::XMQuaternionIsUnit(OrientationA));

		// Transform frustum A into the space of the frustum B in order to
		// minimize the number of transforms we have to do.
		OriginA = XMVector3InverseRotate(XMVectorSubtract(OriginA, OriginB), OrientationB);
		OrientationA = XMQuaternionMultiply(OrientationA, XMQuaternionConjugate(OrientationB));

		// Build the corners of frustum A (in the local space of B).
		XMVECTOR RightTopA = XMVectorSet(fr.RightSlope, fr.TopSlope, 1.0f, 0.0f);
		XMVECTOR RightBottomA = XMVectorSet(fr.RightSlope, fr.BottomSlope, 1.0f, 0.0f);
		XMVECTOR LeftTopA = XMVectorSet(fr.LeftSlope, fr.TopSlope, 1.0f, 0.0f);
		XMVECTOR LeftBottomA = XMVectorSet(fr.LeftSlope, fr.BottomSlope, 1.0f, 0.0f);
		XMVECTOR NearA = XMVectorReplicatePtr(&fr.Near);
		XMVECTOR FarA = XMVectorReplicatePtr(&fr.Far);

		RightTopA = XMVector3Rotate(RightTopA, OrientationA);
		RightBottomA = XMVector3Rotate(RightBottomA, OrientationA);
		LeftTopA = XMVector3Rotate(LeftTopA, OrientationA);
		LeftBottomA = XMVector3Rotate(LeftBottomA, OrientationA);

		XMVECTOR CornersA[BoundingFrustum::CORNER_COUNT];
		CornersA[0] = XMVectorMultiplyAdd(RightTopA, NearA, OriginA);
		CornersA[1] = XMVectorMultiplyAdd(RightBottomA, NearA, OriginA);
		CornersA[2] = XMVectorMultiplyAdd(LeftTopA, NearA, OriginA);
		CornersA[3] = XMVectorMultiplyAdd(LeftBottomA, NearA, OriginA);
		CornersA[4] = XMVectorMultiplyAdd(RightTopA, FarA, OriginA);
		CornersA[5] = XMVectorMultiplyAdd(RightBottomA, FarA, OriginA);
		CornersA[6] = XMVectorMultiplyAdd(LeftTopA, FarA, OriginA);
		CornersA[7] = XMVectorMultiplyAdd(LeftBottomA, FarA, OriginA);

		// Check frustum A against each plane of frustum B.
		XMVECTOR Outside = XMVectorFalseInt();
		XMVECTOR InsideAll = XMVectorTrueInt();

		for (size_t i = 0; i < 6; ++i)
		{
			// Find the min/max projection of the frustum onto the plane normal.
			XMVECTOR Min, Max;

			Min = Max = XMVector3Dot(AxisB[i], CornersA[0]);

			for (size_t j = 1; j < BoundingFrustum::CORNER_COUNT; j++)
			{
				XMVECTOR Temp = XMVector3Dot(AxisB[i], CornersA[j]);
				Min = XMVectorMin(Min, Temp);
				Max = XMVectorMax(Max, Temp);
			}

			// Outside the plane?
			Outside = XMVectorOrInt(Outside, XMVectorGreater(Min, PlaneDistB[i]));

			// Fully inside the plane?
			InsideAll = XMVectorAndInt(InsideAll, XMVectorLessOrEqual(Max, PlaneDistB[i]));
		}

		// If the frustum A is outside any of the planes of frustum B it is outside.
		if (XMVector4EqualInt(Outside, XMVectorTrueInt()))
			return false;

		// If frustum A is inside all planes of frustum B it is fully inside.
		if (XMVector4EqualInt(InsideAll, XMVectorTrueInt()))
			return true;

		// Build the corners of frustum B.
		XMVECTOR RightTopB = XMVectorSet(m_frustum.RightSlope, m_frustum.TopSlope, 1.0f, 0.0f);
		XMVECTOR RightBottomB = XMVectorSet(m_frustum.RightSlope, m_frustum.BottomSlope, 1.0f, 0.0f);
		XMVECTOR LeftTopB = XMVectorSet(m_frustum.LeftSlope, m_frustum.TopSlope, 1.0f, 0.0f);
		XMVECTOR LeftBottomB = XMVectorSet(m_frustum.LeftSlope, m_frustum.BottomSlope, 1.0f, 0.0f);
		XMVECTOR NearB = XMVectorReplicatePtr(&m_frustum.Near);
		XMVECTOR FarB = XMVectorReplicatePtr(&m_frustum.Far);

		XMVECTOR CornersB[BoundingFrustum::CORNER_COUNT];
		CornersB[0] = XMVectorMultiply(RightTopB, NearB);
		CornersB[1] = XMVectorMultiply(RightBottomB, NearB);
		CornersB[2] = XMVectorMultiply(LeftTopB, NearB);
		CornersB[3] = XMVectorMultiply(LeftBottomB, NearB);
		CornersB[4] = XMVectorMultiply(RightTopB, FarB);
		CornersB[5] = XMVectorMultiply(RightBottomB, FarB);
		CornersB[6] = XMVectorMultiply(LeftTopB, FarB);
		CornersB[7] = XMVectorMultiply(LeftBottomB, FarB);

		// Build the planes of frustum A (in the local space of B).
		XMVECTOR AxisA[6];
		XMVECTOR PlaneDistA[6];

		AxisA[0] = XMVectorSet(0.0f, 0.0f, -1.0f, 0.0f);
		AxisA[1] = XMVectorSet(0.0f, 0.0f, 1.0f, 0.0f);
		AxisA[2] = XMVectorSet(1.0f, 0.0f, -fr.RightSlope, 0.0f);
		AxisA[3] = XMVectorSet(-1.0f, 0.0f, fr.LeftSlope, 0.0f);
		AxisA[4] = XMVectorSet(0.0f, 1.0f, -fr.TopSlope, 0.0f);
		AxisA[5] = XMVectorSet(0.0f, -1.0f, fr.BottomSlope, 0.0f);

		AxisA[0] = XMVector3Rotate(AxisA[0], OrientationA);
		AxisA[1] = XMVectorNegate(AxisA[0]);
		AxisA[2] = XMVector3Rotate(AxisA[2], OrientationA);
		AxisA[3] = XMVector3Rotate(AxisA[3], OrientationA);
		AxisA[4] = XMVector3Rotate(AxisA[4], OrientationA);
		AxisA[5] = XMVector3Rotate(AxisA[5], OrientationA);

		PlaneDistA[0] = XMVector3Dot(AxisA[0], CornersA[0]);  // Re-use corner on near plane.
		PlaneDistA[1] = XMVector3Dot(AxisA[1], CornersA[4]);  // Re-use corner on far plane.
		PlaneDistA[2] = XMVector3Dot(AxisA[2], OriginA);
		PlaneDistA[3] = XMVector3Dot(AxisA[3], OriginA);
		PlaneDistA[4] = XMVector3Dot(AxisA[4], OriginA);
		PlaneDistA[5] = XMVector3Dot(AxisA[5], OriginA);

		// Check each axis of frustum A for a seperating plane (5).
		for (size_t i = 0; i < 6; ++i)
		{
			// Find the minimum projection of the frustum onto the plane normal.
			XMVECTOR Min;

			Min = XMVector3Dot(AxisA[i], CornersB[0]);

			for (size_t j = 1; j < BoundingFrustum::CORNER_COUNT; j++)
			{
				XMVECTOR Temp = XMVector3Dot(AxisA[i], CornersB[j]);
				Min = XMVectorMin(Min, Temp);
			}

			// Outside the plane?
			Outside = XMVectorOrInt(Outside, XMVectorGreater(Min, PlaneDistA[i]));
		}

		// If the frustum B is outside any of the planes of frustum A it is outside.
		if (XMVector4EqualInt(Outside, XMVectorTrueInt()))
			return false;

		// Check edge/edge axes (6 * 6).
		XMVECTOR FrustumEdgeAxisA[6];
		FrustumEdgeAxisA[0] = RightTopA;
		FrustumEdgeAxisA[1] = RightBottomA;
		FrustumEdgeAxisA[2] = LeftTopA;
		FrustumEdgeAxisA[3] = LeftBottomA;
		FrustumEdgeAxisA[4] = XMVectorSubtract(RightTopA, LeftTopA);
		FrustumEdgeAxisA[5] = XMVectorSubtract(LeftBottomA, LeftTopA);

		XMVECTOR FrustumEdgeAxisB[6];
		FrustumEdgeAxisB[0] = RightTopB;
		FrustumEdgeAxisB[1] = RightBottomB;
		FrustumEdgeAxisB[2] = LeftTopB;
		FrustumEdgeAxisB[3] = LeftBottomB;
		FrustumEdgeAxisB[4] = XMVectorSubtract(RightTopB, LeftTopB);
		FrustumEdgeAxisB[5] = XMVectorSubtract(LeftBottomB, LeftTopB);

		for (size_t i = 0; i < 6; ++i)
		{
			for (size_t j = 0; j < 6; j++)
			{
				// Compute the axis we are going to test.
				XMVECTOR Axis = XMVector3Cross(FrustumEdgeAxisA[i], FrustumEdgeAxisB[j]);

				// Find the min/max values of the projection of both frustums onto the axis.
				XMVECTOR MinA, MaxA;
				XMVECTOR MinB, MaxB;

				MinA = MaxA = XMVector3Dot(Axis, CornersA[0]);
				MinB = MaxB = XMVector3Dot(Axis, CornersB[0]);

				for (size_t k = 1; k < BoundingFrustum::CORNER_COUNT; k++)
				{
					XMVECTOR TempA = XMVector3Dot(Axis, CornersA[k]);
					MinA = XMVectorMin(MinA, TempA);
					MaxA = XMVectorMax(MaxA, TempA);

					XMVECTOR TempB = XMVector3Dot(Axis, CornersB[k]);
					MinB = XMVectorMin(MinB, TempB);
					MaxB = XMVectorMax(MaxB, TempB);
				}

				// if (MinA > MaxB || MinB > MaxA) reject
				Outside = XMVectorOrInt(Outside, XMVectorGreater(MinA, MaxB));
				Outside = XMVectorOrInt(Outside, XMVectorGreater(MinB, MaxA));
			}
		}

		// If there is a seperating plane, then the frustums do not intersect.
		if (XMVector4EqualInt(Outside, XMVectorTrueInt()))
			return false;

		// If we did not find a separating plane then the frustums intersect.
		return true;
	}

	Optional<float> ViewFrustum::intersects(const Ray& ray) const noexcept
	{
		return ray.intersects(*this);
	}

	bool ViewFrustum::contains(const Vec3& point) const noexcept
	{
		return intersects(point);
	}

	bool ViewFrustum::contains(const Triangle3D& triangle) const noexcept
	{
		using namespace DirectX;

		// Load origin and orientation of the frustum.
		XMVECTOR vOrigin = XMLoadFloat3(&m_frustum.Origin);
		XMVECTOR vOrientation = XMLoadFloat4(&m_frustum.Orientation);

		// Create 6 planes (do it inline to encourage use of registers)
		XMVECTOR NearPlane = XMVectorSet(0.0f, 0.0f, -1.0f, m_frustum.Near);
		NearPlane = DirectX::Internal::XMPlaneTransform(NearPlane, vOrientation, vOrigin);
		NearPlane = XMPlaneNormalize(NearPlane);

		XMVECTOR FarPlane = XMVectorSet(0.0f, 0.0f, 1.0f, -m_frustum.Far);
		FarPlane = DirectX::Internal::XMPlaneTransform(FarPlane, vOrientation, vOrigin);
		FarPlane = XMPlaneNormalize(FarPlane);

		XMVECTOR RightPlane = XMVectorSet(1.0f, 0.0f, -m_frustum.RightSlope, 0.0f);
		RightPlane = DirectX::Internal::XMPlaneTransform(RightPlane, vOrientation, vOrigin);
		RightPlane = XMPlaneNormalize(RightPlane);

		XMVECTOR LeftPlane = XMVectorSet(-1.0f, 0.0f, m_frustum.LeftSlope, 0.0f);
		LeftPlane = DirectX::Internal::XMPlaneTransform(LeftPlane, vOrientation, vOrigin);
		LeftPlane = XMPlaneNormalize(LeftPlane);

		XMVECTOR TopPlane = XMVectorSet(0.0f, 1.0f, -m_frustum.TopSlope, 0.0f);
		TopPlane = DirectX::Internal::XMPlaneTransform(TopPlane, vOrientation, vOrigin);
		TopPlane = XMPlaneNormalize(TopPlane);

		XMVECTOR BottomPlane = XMVectorSet(0.0f, -1.0f, m_frustum.BottomSlope, 0.0f);
		BottomPlane = DirectX::Internal::XMPlaneTransform(BottomPlane, vOrientation, vOrigin);
		BottomPlane = XMPlaneNormalize(BottomPlane);

		const auto result = TriangleTests::ContainedBy(triangle.p0, triangle.p1, triangle.p2, NearPlane, FarPlane, RightPlane, LeftPlane, TopPlane, BottomPlane);
		return (result == DirectX::ContainmentType::CONTAINS);
	}

	bool ViewFrustum::contains(const Sphere& sphere) const noexcept
	{
		using namespace DirectX;
		const auto sh = detail::FromSphere(sphere);

		// Load origin and orientation of the frustum.
		XMVECTOR vOrigin = XMLoadFloat3(&m_frustum.Origin);
		XMVECTOR vOrientation = XMLoadFloat4(&m_frustum.Orientation);

		// Create 6 planes (do it inline to encourage use of registers)
		XMVECTOR NearPlane = XMVectorSet(0.0f, 0.0f, -1.0f, m_frustum.Near);
		NearPlane = DirectX::Internal::XMPlaneTransform(NearPlane, vOrientation, vOrigin);
		NearPlane = XMPlaneNormalize(NearPlane);

		XMVECTOR FarPlane = XMVectorSet(0.0f, 0.0f, 1.0f, -m_frustum.Far);
		FarPlane = DirectX::Internal::XMPlaneTransform(FarPlane, vOrientation, vOrigin);
		FarPlane = XMPlaneNormalize(FarPlane);

		XMVECTOR RightPlane = XMVectorSet(1.0f, 0.0f, -m_frustum.RightSlope, 0.0f);
		RightPlane = DirectX::Internal::XMPlaneTransform(RightPlane, vOrientation, vOrigin);
		RightPlane = XMPlaneNormalize(RightPlane);

		XMVECTOR LeftPlane = XMVectorSet(-1.0f, 0.0f, m_frustum.LeftSlope, 0.0f);
		LeftPlane = DirectX::Internal::XMPlaneTransform(LeftPlane, vOrientation, vOrigin);
		LeftPlane = XMPlaneNormalize(LeftPlane);

		XMVECTOR TopPlane = XMVectorSet(0.0f, 1.0f, -m_frustum.TopSlope, 0.0f);
		TopPlane = DirectX::Internal::XMPlaneTransform(TopPlane, vOrientation, vOrigin);
		TopPlane = XMPlaneNormalize(TopPlane);

		XMVECTOR BottomPlane = XMVectorSet(0.0f, -1.0f, m_frustum.BottomSlope, 0.0f);
		BottomPlane = DirectX::Internal::XMPlaneTransform(BottomPlane, vOrientation, vOrigin);
		BottomPlane = XMPlaneNormalize(BottomPlane);

		const auto result = sh.ContainedBy(NearPlane, FarPlane, RightPlane, LeftPlane, TopPlane, BottomPlane);
		return (result == DirectX::ContainmentType::CONTAINS);
	}

	bool ViewFrustum::contains(const Box& box) const noexcept
	{
		using namespace DirectX;
		const auto b = detail::FromOrientedBox(box);

		// Load origin and orientation of the frustum.
		XMVECTOR vOrigin = XMLoadFloat3(&m_frustum.Origin);
		XMVECTOR vOrientation = XMLoadFloat4(&m_frustum.Orientation);

		// Create 6 planes (do it inline to encourage use of registers)
		XMVECTOR NearPlane = XMVectorSet(0.0f, 0.0f, -1.0f, m_frustum.Near);
		NearPlane = DirectX::Internal::XMPlaneTransform(NearPlane, vOrientation, vOrigin);
		NearPlane = XMPlaneNormalize(NearPlane);

		XMVECTOR FarPlane = XMVectorSet(0.0f, 0.0f, 1.0f, -m_frustum.Far);
		FarPlane = DirectX::Internal::XMPlaneTransform(FarPlane, vOrientation, vOrigin);
		FarPlane = XMPlaneNormalize(FarPlane);

		XMVECTOR RightPlane = XMVectorSet(1.0f, 0.0f, -m_frustum.RightSlope, 0.0f);
		RightPlane = DirectX::Internal::XMPlaneTransform(RightPlane, vOrientation, vOrigin);
		RightPlane = XMPlaneNormalize(RightPlane);

		XMVECTOR LeftPlane = XMVectorSet(-1.0f, 0.0f, m_frustum.LeftSlope, 0.0f);
		LeftPlane = DirectX::Internal::XMPlaneTransform(LeftPlane, vOrientation, vOrigin);
		LeftPlane = XMPlaneNormalize(LeftPlane);

		XMVECTOR TopPlane = XMVectorSet(0.0f, 1.0f, -m_frustum.TopSlope, 0.0f);
		TopPlane = DirectX::Internal::XMPlaneTransform(TopPlane, vOrientation, vOrigin);
		TopPlane = XMPlaneNormalize(TopPlane);

		XMVECTOR BottomPlane = XMVectorSet(0.0f, -1.0f, m_frustum.BottomSlope, 0.0f);
		BottomPlane = DirectX::Internal::XMPlaneTransform(BottomPlane, vOrientation, vOrigin);
		BottomPlane = XMPlaneNormalize(BottomPlane);

		const auto result = b.ContainedBy(NearPlane, FarPlane, RightPlane, LeftPlane, TopPlane, BottomPlane);
		return (result == DirectX::ContainmentType::CONTAINS);
	}

	bool ViewFrustum::contains(const OrientedBox& box) const noexcept
	{
		using namespace DirectX;
		const auto b = detail::FromOrientedBox(box);

		// Load origin and orientation of the frustum.
		XMVECTOR vOrigin = XMLoadFloat3(&m_frustum.Origin);
		XMVECTOR vOrientation = XMLoadFloat4(&m_frustum.Orientation);

		// Create 6 planes (do it inline to encourage use of registers)
		XMVECTOR NearPlane = XMVectorSet(0.0f, 0.0f, -1.0f, m_frustum.Near);
		NearPlane = DirectX::Internal::XMPlaneTransform(NearPlane, vOrientation, vOrigin);
		NearPlane = XMPlaneNormalize(NearPlane);

		XMVECTOR FarPlane = XMVectorSet(0.0f, 0.0f, 1.0f, -m_frustum.Far);
		FarPlane = DirectX::Internal::XMPlaneTransform(FarPlane, vOrientation, vOrigin);
		FarPlane = XMPlaneNormalize(FarPlane);

		XMVECTOR RightPlane = XMVectorSet(1.0f, 0.0f, -m_frustum.RightSlope, 0.0f);
		RightPlane = DirectX::Internal::XMPlaneTransform(RightPlane, vOrientation, vOrigin);
		RightPlane = XMPlaneNormalize(RightPlane);

		XMVECTOR LeftPlane = XMVectorSet(-1.0f, 0.0f, m_frustum.LeftSlope, 0.0f);
		LeftPlane = DirectX::Internal::XMPlaneTransform(LeftPlane, vOrientation, vOrigin);
		LeftPlane = XMPlaneNormalize(LeftPlane);

		XMVECTOR TopPlane = XMVectorSet(0.0f, 1.0f, -m_frustum.TopSlope, 0.0f);
		TopPlane = DirectX::Internal::XMPlaneTransform(TopPlane, vOrientation, vOrigin);
		TopPlane = XMPlaneNormalize(TopPlane);

		XMVECTOR BottomPlane = XMVectorSet(0.0f, -1.0f, m_frustum.BottomSlope, 0.0f);
		BottomPlane = DirectX::Internal::XMPlaneTransform(BottomPlane, vOrientation, vOrigin);
		BottomPlane = XMPlaneNormalize(BottomPlane);

		const auto result = b.ContainedBy(NearPlane, FarPlane, RightPlane, LeftPlane, TopPlane, BottomPlane);
		return (result == DirectX::ContainmentType::CONTAINS);
	}

	bool ViewFrustum::contains(const ViewFrustum& frustum) const noexcept
	{
		using namespace DirectX;
		const auto& fr = frustum.getData();

		// Load origin and orientation of the frustum.
		XMVECTOR vOrigin = XMLoadFloat3(&m_frustum.Origin);
		XMVECTOR vOrientation = XMLoadFloat4(&m_frustum.Orientation);

		// Create 6 planes (do it inline to encourage use of registers)
		XMVECTOR NearPlane = XMVectorSet(0.0f, 0.0f, -1.0f, m_frustum.Near);
		NearPlane = DirectX::Internal::XMPlaneTransform(NearPlane, vOrientation, vOrigin);
		NearPlane = XMPlaneNormalize(NearPlane);

		XMVECTOR FarPlane = XMVectorSet(0.0f, 0.0f, 1.0f, -m_frustum.Far);
		FarPlane = DirectX::Internal::XMPlaneTransform(FarPlane, vOrientation, vOrigin);
		FarPlane = XMPlaneNormalize(FarPlane);

		XMVECTOR RightPlane = XMVectorSet(1.0f, 0.0f, -m_frustum.RightSlope, 0.0f);
		RightPlane = DirectX::Internal::XMPlaneTransform(RightPlane, vOrientation, vOrigin);
		RightPlane = XMPlaneNormalize(RightPlane);

		XMVECTOR LeftPlane = XMVectorSet(-1.0f, 0.0f, m_frustum.LeftSlope, 0.0f);
		LeftPlane = DirectX::Internal::XMPlaneTransform(LeftPlane, vOrientation, vOrigin);
		LeftPlane = XMPlaneNormalize(LeftPlane);

		XMVECTOR TopPlane = XMVectorSet(0.0f, 1.0f, -m_frustum.TopSlope, 0.0f);
		TopPlane = DirectX::Internal::XMPlaneTransform(TopPlane, vOrientation, vOrigin);
		TopPlane = XMPlaneNormalize(TopPlane);

		XMVECTOR BottomPlane = XMVectorSet(0.0f, -1.0f, m_frustum.BottomSlope, 0.0f);
		BottomPlane = DirectX::Internal::XMPlaneTransform(BottomPlane, vOrientation, vOrigin);
		BottomPlane = XMPlaneNormalize(BottomPlane);

		const auto result = fr.ContainedBy(NearPlane, FarPlane, RightPlane, LeftPlane, TopPlane, BottomPlane);
		return (result == DirectX::ContainmentType::CONTAINS);
	}
	*/

	//-------------------------------------------------------------------------------------

	Vec3 ViewFrustum::getOrigin() const noexcept
	{
		return{
			m_frustum.Origin.x,
			m_frustum.Origin.y,
			m_frustum.Origin.z
		};
	}

	Quaternion ViewFrustum::getOrientation() const noexcept
	{
		return{
			m_frustum.Orientation.x,
			m_frustum.Orientation.y,
			m_frustum.Orientation.z,
			m_frustum.Orientation.w,
		};
	}

	Sphere ViewFrustum::computeBoundingSphere() const noexcept
	{
		DirectX::BoundingSphere result;
		DirectX::BoundingSphere::CreateFromFrustum(result, m_frustum);
		return detail::ToSphere(result);
	}

	const ViewFrustum& ViewFrustum::drawFrame(const ColorF& color) const
	{
		const std::array<Vec3, 8> c = getCorners();

		Line3D{ c[0], c[1] }.draw(color);
		Line3D{ c[1], c[3] }.draw(color);
		Line3D{ c[3], c[2] }.draw(color);
		Line3D{ c[2], c[0] }.draw(color);

		Line3D{ c[0], c[4] }.draw(color);
		Line3D{ c[1], c[5] }.draw(color);
		Line3D{ c[2], c[6] }.draw(color);
		Line3D{ c[3], c[7] }.draw(color);

		Line3D{ c[4], c[5] }.draw(color);
		Line3D{ c[5], c[7] }.draw(color);
		Line3D{ c[7], c[6] }.draw(color);
		Line3D{ c[6], c[4] }.draw(color);

		return *this;
	}

	const DirectX::BoundingFrustum& ViewFrustum::getData() const noexcept
	{
		return m_frustum;
	}
}
