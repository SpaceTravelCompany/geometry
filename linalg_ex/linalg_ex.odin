package linalg_ex

import "base:intrinsics"
import "core:math"
import "core:math/linalg"

import "shared:utils_private"


Recti32 :: Rect_(i32)
Rectu32 :: Rect_(u32)
Rectf32 :: Rect_(f32)

CenterPtPos :: enum {
	Center,
	Left,
	Right,
	TopLeft,
	Top,
	TopRight,
	BottomLeft,
	Bottom,
	BottomRight,
}

PointInPolygonResult :: enum u8 {
	On,
	Outside,
	Inside,
}

Rect_ :: struct($T: typeid) where intrinsics.type_is_numeric(T) {
	left:   T,
	right:  T,
	top:    T,
	bottom: T,
}


RectInit :: #force_inline proc "contextless" (left: $T, right: T, top: T, bottom: T) -> Rect_(T) {
	res: Rect_(T)
	res.left = left
	res.right = right
	res.top = top
	res.bottom = bottom
	return res
}

CheckRect :: #force_inline proc "contextless" (
	_pts: [4][$N]$T,
) -> bool where N >= 2 &&
	intrinsics.type_is_numeric(T) {
	return(
		!(!((_pts[0].y) == (_pts[1].y)) ||
			!((_pts[2].y) == (_pts[3].y)) ||
			!((_pts[0].x) == (_pts[2].x)) ||
			!((_pts[1].x) == (_pts[3].x))) \
	)
}

RectMulMatrix :: proc "contextless" (_r: Rectf32, _mat: linalg.Matrix4x4f32) -> (Rectf32, bool) {
	tps := __RectMulMatrix(_r, _mat)

	if CheckRect(tps) do return {}, false

	return Rectf32{left = tps[0].x, right = tps[3].x, top = tps[0].y, bottom = tps[3].y}, true
}
RectDivMatrix :: proc "contextless" (_r: Rectf32, _mat: linalg.Matrix4x4f32) -> (Rectf32, bool) {
	// Apply inverse transform (Rectf32 / M == Rectf32 * inverse(M))
	return RectMulMatrix(_r, linalg.inverse(_mat))
}

@(private)
__RectMulMatrix :: proc "contextless" (
	_r: Rectf32,
	_mat: linalg.Matrix4x4f32,
) -> [4]linalg.Vector4f32 {
	// Transform 4 corners, then rebuild AABB.
	x0 := _r.left
	y0 := _r.top
	x1 := _r.right
	y1 := _r.bottom

	p0 := linalg.Vector4f32{x0, y0, 0, 1}
	p1 := linalg.Vector4f32{x1, y0, 0, 1}
	p2 := linalg.Vector4f32{x0, y1, 0, 1}
	p3 := linalg.Vector4f32{x1, y1, 0, 1}

	t0 := linalg.mul(_mat, p0)
	t1 := linalg.mul(_mat, p1)
	t2 := linalg.mul(_mat, p2)
	t3 := linalg.mul(_mat, p3)

	// Homogeneous divide if needed (safe for ortho/TRS where w == 1).
	tx0, ty0 := t0.x, t0.y
	tx1, ty1 := t1.x, t1.y
	tx2, ty2 := t2.x, t2.y
	tx3, ty3 := t3.x, t3.y

	if t0.w != 0 {tx0 /= t0.w; ty0 /= t0.w}
	if t1.w != 0 {tx1 /= t1.w; ty1 /= t1.w}
	if t2.w != 0 {tx2 /= t2.w; ty2 /= t2.w}
	if t3.w != 0 {tx3 /= t3.w; ty3 /= t3.w}

	return [4]linalg.Vector4f32{t0, t1, t2, t3}
}

/*
Multiplies a rectangle or a polygon by a matrix.
if input is a rectangle, and converted result is not a rectangle, it is converted to a polygon.
if input is a polygon, return the polygon.
if return value is the polygon, it is allocated using allocator.

Inputs:
- _a: Areaf32(Rectangle or polygon) to multiply
- _mat: matrix44 to multiply
- allocator: Allocator to use

Returns:
- Areaf32 multiplied by the matrix
*/
AreaMulMatrix :: proc(
	_a: Areaf32,
	_mat: linalg.Matrix4x4f32,
	allocator := context.allocator,
) -> Areaf32 {
	switch &n in _a {
	case Rectf32:
		res := __RectMulMatrix(n, _mat)
		if CheckRect(res) {
			minX := min(res[0].x, res[3].x)
			maxX := max(res[0].x, res[3].x)
			minY := min(res[0].y, res[3].y)
			maxY := max(res[0].y, res[3].y)
			return Rectf32{left = minX, right = maxX, top = maxY, bottom = minY}
		}
		res2 := utils_private.makeNonZeroedSlice([][2]f32, 4, allocator)
		res2[0] = res[0].xy
		res2[1] = res[1].xy
		res2[2] = res[3].xy
		res2[3] = res[2].xy
		return res2
	case [][2]f32:
		return __PolyMulMatrix(n, _mat, allocator)
	}
	return {}
}

@(private)
__PolyMulMatrix :: proc(
	_p: [][$N]f32,
	_mat: linalg.Matrix4x4f32,
	allocator := context.allocator,
) -> Areaf32 where N >=
	2 {
	res: [][2]f32 = utils_private.makeNonZeroedSlice([][2]f32, len(_p), allocator)
	for i in 0 ..< len(res) {
		when N == 4 {
			r := linalg.mul(_mat, _p[i])
		} else when N == 2 {
			r := linalg.mul(_mat, linalg.Vector4f32{_p[i].x, _p[i].y, 0.0, 1.0})
		} else when N == 3 {
			r := linalg.mul(_mat, linalg.Vector4f32{_p[i].x, _p[i].y, _p[i].z, 1.0})
		} else {
			#panic("not implemented")
		}
		if r.w != 0 {r.x /= r.w; r.y /= r.w}
		res[i] = r.xy
	}
	return res
}
RectLeftTop :: #force_inline proc "contextless" (_r: Rect_($T)) -> [2]T {
	return [2]T{_r.left, _r.top}
}
RectRightBottom :: #force_inline proc "contextless" (_r: Rect_($T)) -> [2]T {
	return [2]T{_r.right, _r.bottom}
}
RectAnd :: #force_inline proc "contextless" (
	_r1: Rect_($T),
	_r2: Rect_(T),
) -> Rect_(T) #no_bounds_check {
	res: Rect_(T)
	res.left = ((_r1.left) > (_r2.left) ? (_r1.left) : (_r2.left))
	res.right = ((_r1.right) < (_r2.right) ? (_r1.right) : (_r2.right))
	if ((res.right) < (res.left)) do return {}

	r1Top := ((_r1.top) > (_r1.bottom) ? (_r1.top) : (_r1.bottom))
	r1Bottom := ((_r1.top) < (_r1.bottom) ? (_r1.top) : (_r1.bottom))
	r2Top := ((_r2.top) > (_r2.bottom) ? (_r2.top) : (_r2.bottom))
	r2Bottom := ((_r2.top) < (_r2.bottom) ? (_r2.top) : (_r2.bottom))

	yTop := ((r1Top) < (r2Top) ? (r1Top) : (r2Top))
	yBottom := ((r1Bottom) > (r2Bottom) ? (r1Bottom) : (r2Bottom))

	if ((_r1.top) >= (_r1.bottom)) {
		res.top = yTop
		res.bottom = yBottom
	} else {
		res.top = yBottom
		res.bottom = yTop
	}
	return res
}
RectOr :: #force_inline proc "contextless" (
	_r1: Rect_($T),
	_r2: Rect_(T),
) -> Rect_(T) #no_bounds_check {
	res: Rect_(T)
	res.left = ((_r1.left) < (_r2.left) ? (_r1.left) : (_r2.left))
	res.right = ((_r1.right) > (_r2.right) ? (_r1.right) : (_r2.right))

	r1Top := ((_r1.top) > (_r1.bottom) ? (_r1.top) : (_r1.bottom))
	r1Bottom := ((_r1.top) < (_r1.bottom) ? (_r1.top) : (_r1.bottom))
	r2Top := ((_r2.top) > (_r2.bottom) ? (_r2.top) : (_r2.bottom))
	r2Bottom := ((_r2.top) < (_r2.bottom) ? (_r2.top) : (_r2.bottom))

	yTop := ((r1Top) > (r2Top) ? (r1Top) : (r2Top))
	yBottom := ((r1Bottom) < (r2Bottom) ? (r1Bottom) : (r2Bottom))

	if ((_r1.top) >= (_r1.bottom)) {
		res.top = yTop
		res.bottom = yBottom
	} else {
		res.top = yBottom
		res.bottom = yTop
	}

	return res
}
RectPointIn :: #force_inline proc "contextless" (_r: Rect_($T), p: [2]T) -> bool #no_bounds_check {
	return(
		p.x >= _r.left &&
		p.x <= _r.right &&
		(_r.top > _r.bottom ? (p.y <= _r.top && p.y >= _r.bottom) : (p.y >= _r.top && p.y <= _r.bottom)) \
	)
}

// Polygon-polygon overlap: vertex containment or edge intersection.
PolygonOverlapsPolygon :: proc "contextless" (
	poly1, poly2: [][2]$T,
) -> bool where intrinsics.type_is_float(T) {
	if len(poly1) < 3 || len(poly2) < 3 do return false
	for p in poly1 {
		if PointInPolygon(p, poly2) == .Inside do return true
	}
	for p in poly2 {
		if PointInPolygon(p, poly1) == .Inside do return true
	}
	for i in 0 ..< len(poly1) {
		a1 := poly1[i]
		b1 := poly1[(i + 1) % len(poly1)]
		for j in 0 ..< len(poly2) {
			a2 := poly2[j]
			b2 := poly2[(j + 1) % len(poly2)]
			if LinesIntersect3(a1, b1, a2, b2) == .intersect do return true
		}
	}
	return false
}

RectMove :: #force_inline proc "contextless" (
	_r: Rect_($T),
	p: [2]T,
) -> Rect_(T) #no_bounds_check {
	res: Rect_(T)
	res.left = _r.left + p.x
	res.top = _r.top + p.y
	res.right = _r.right + p.x
	res.bottom = _r.bottom + p.y
	return res
}

PointInTriangle :: proc "contextless" (
	p: [2]$T,
	a: [2]T,
	b: [2]T,
	c: [2]T,
) -> bool where intrinsics.type_is_float(T) {
	zero := T(0)
	x0 := ((c.x) - (a.x))
	y0 := ((c.y) - (a.y))
	x1 := ((b.x) - (a.x))
	y1 := ((b.y) - (a.y))
	x2 := ((p.x) - (a.x))
	y2 := ((p.y) - (a.y))

	dot00 := ((((x0) * (x0))) + (((y0) * (y0))))
	dot01 := ((((x0) * (x1))) + (((y0) * (y1))))
	dot02 := ((((x0) * (x2))) + (((y0) * (y2))))
	dot11 := ((((x1) * (x1))) + (((y1) * (y1))))
	dot12 := ((((x1) * (x2))) + (((y1) * (y2))))
	denominator := ((((dot00) * (dot11))) - (((dot01) * (dot01))))
	if ((denominator) == (zero)) do return false

	u := ((((dot11) * (dot02))) - (((dot01) * (dot12))))
	v := ((((dot00) * (dot12))) - (((dot01) * (dot02))))

	if ((denominator) > (zero)) {
		return ((u) > (zero)) && ((v) > (zero)) && ((((u) + (v))) < (denominator))
	}
	return ((u) < (zero)) && ((v) < (zero)) && ((((u) + (v))) > (denominator))
}

epsilon :: proc "contextless" ($T: typeid) -> T where intrinsics.type_is_float(T) {
	when T == f32 {
		return T(math.F32_EPSILON)
	} else when T == f64 {
		return T(math.F64_EPSILON)
	} else {
		return T(math.F16_EPSILON)
	}
}

PointInLine :: proc "contextless" (
	p: [2]$T,
	l0: [2]T,
	l1: [2]T,
) -> (
	bool,
	T,
) where intrinsics.type_is_float(T) {
	minX := ((l0.x) < (l1.x) ? (l0.x) : (l1.x))
	maxX := ((l0.x) > (l1.x) ? (l0.x) : (l1.x))
	minY := ((l0.y) < (l1.y) ? (l0.y) : (l1.y))
	maxY := ((l0.y) > (l1.y) ? (l0.y) : (l1.y))
	a := ((((l0.y) - (l1.y))) / (((l0.x) - (l1.x))))
	b := ((l0.y) - (((a) * (l0.x))))
	pY := ((((a) * (p.x))) + (b))
	inBbox := ((p.x) >= (minX)) && ((p.x) <= (maxX)) && ((p.y) >= (minY)) && ((p.y) <= (maxY))
	t := ((((p.x) - (minX))) / (((maxX) - (minX))))

	onLine := (math.abs((p.y) - (pY)) <= epsilon(T) * T(16))
	return onLine && inBbox, t
}

SubdivLine :: proc "contextless" (
	pts: [2][2]$T,
	subdiv: T,
) -> (
	pt01: [2]T,
) where intrinsics.type_is_float(T) {
	pt01 = linalg.lerp(pts[0], pts[1], subdiv)
	return
}

SubdivQuadraticBezier :: proc "contextless" (
	pts: [3][2]$T,
	subdiv: T,
) -> (
	pt1, pt12, pt2: [2]T,
) where intrinsics.type_is_float(T) {
	pt1 = linalg.lerp(pts[0], pts[1], subdiv)
	pt2 = linalg.lerp(pts[1], pts[2], subdiv)
	pt12 = linalg.lerp(pt1, pt2, subdiv)
	return
}

SubdivCubicBezier :: proc "contextless" (
	pts: [4][2]$T,
	subdiv: T,
) -> (
	c0, c1, m, d0, d1: [2]T,
) where intrinsics.type_is_float(T) {
	p01 := linalg.lerp(pts[0], pts[1], subdiv)
	p12 := linalg.lerp(pts[1], pts[2], subdiv)
	p23 := linalg.lerp(pts[2], pts[3], subdiv)

	p012 := linalg.lerp(p01, p12, subdiv)
	p123 := linalg.lerp(p12, p23, subdiv)

	c0 = p01
	c1 = p012
	m = linalg.lerp(p012, p123, subdiv)
	d0 = p123
	d1 = p23
	return
}

PointDeltaInLine :: proc "contextless" (
	p: [2]$T,
	l0: [2]T,
	l1: [2]T,
) -> T where intrinsics.type_is_float(T) {
	pp := NearestPointBetweenPointAndLine(p, l0, l1)
	t := linalg.lerp(l0.x, l1.x, pp.x)
	return t
}

PointInVector :: proc "contextless" (
	p: [2]$T,
	v0: [2]T,
	v1: [2]T,
) -> (
	bool,
	T,
) where intrinsics.type_is_float(T) {
	zero := T(0)
	a := ((v1.y) - (v0.y))
	b := ((v0.x) - (v1.x))
	c := ((((v1.x) * (v0.y))) + (((v0.x) * (v1.y))))
	res := ((((((a) * (p.x))) + (((b) * (p.y))))) + (c))
	return (math.abs((res) - (zero)) <= epsilon(T) * T(16)), res
}

// > 0: left, < 0: right, == 0: on the line
PointLineLeftOrRight :: #force_inline proc "contextless" (
	p: [2]$T,
	l0: [2]T,
	l1: [2]T,
) -> T where intrinsics.type_is_float(T) {
	return(
		(((((l1.x) - (l0.x))) * (((p.y) - (l0.y))))) -
		(((((p.x) - (l0.x))) * (((l1.y) - (l0.y))))) \
	)
}

PointInPolygon :: proc "contextless" (
	p: [2]$T,
	polygon: [][2]T,
) -> PointInPolygonResult where intrinsics.type_is_float(T) {
	isPointOnSegment :: proc "contextless" (p: [2]$T, p1: [2]T, p2: [2]T) -> bool {
		return(
			CrossProductSign(p1, p2, p) == 0 &&
			((p.x) >= (((p1.x) < (p2.x) ? (p1.x) : (p2.x)))) &&
			((p.x) <= (((p1.x) > (p2.x) ? (p1.x) : (p2.x)))) &&
			((p.y) >= (((p1.y) < (p2.y) ? (p1.y) : (p2.y)))) &&
			((p.y) <= (((p1.y) > (p2.y) ? (p1.y) : (p2.y)))) \
		)
	}
	windingNumber := 0
	for i in 0 ..< len(polygon) {
		p1 := polygon[i]
		p2 := polygon[(i + 1) % len(polygon)]
		if isPointOnSegment(p, p1, p2) do return .On

		if ((p1.y) <= (p.y)) {
			if ((p2.y) > (p.y)) && CrossProductSign(p1, p2, p) > 0 do windingNumber += 1
		} else {
			if ((p2.y) <= (p.y)) && CrossProductSign(p1, p2, p) < 0 do windingNumber -= 1
		}
	}
	return windingNumber != 0 ? .Inside : .Outside
}

CenterPointInPolygon :: proc "contextless" (
	polygon: [][2]$T,
) -> [2]T where intrinsics.type_is_float(T) {
	area: f32 = 0
	p: [2]T = {0, 0}
	for i in 0 ..< len(polygon) {
		j := (i + 1) % len(polygon)
		factor := linalg.vector_cross2(polygon[i], polygon[j])
		area += factor
		p = (polygon[i] + polygon[j]) * math.splat_2(factor) + p
	}
	area = area / 2 * 6
	p /= math.splat_2(area)
	return p
}

GetPolygonOrientation :: proc "contextless" (
	polygon: [][2]$T,
) -> PolyOrientation where intrinsics.type_is_float(T) {
	zero := T(0)
	res: T = zero
	for i in 0 ..< len(polygon) {
		j := (i + 1) % len(polygon)
		factor := ((((polygon[j].x) - (polygon[i].x))) * (((polygon[j].y) + (polygon[i].y))))
		res = ((res) + (factor))
	}
	return ((res) > (zero)) ? .Clockwise : .CounterClockwise
}

PolygonSignedArea :: proc "contextless" (polygon: []linalg.Vector2f32) -> f32 {
	n := len(polygon)
	if n < 3 do return 0

	area := f32(0)
	for i in 0 ..< n {
		j := (i + 1) % n
		area += polygon[i].x * polygon[j].y - polygon[j].x * polygon[i].y
	}
	return area * 0.5
}

LineInPolygon :: proc "contextless" (
	a: [2]$T,
	b: [2]T,
	polygon: [][2]T,
	checkInsideLine := true,
) -> bool where intrinsics.type_is_float(T) {
	//Points a, b must all be inside the polygon so that line a, b and polygon line segments do not intersect, so b does not need to be checked.
	if checkInsideLine && PointInPolygon(a, polygon) do return true

	res: [2]T
	ok: bool
	for i in 0 ..< len(polygon) {
		j := (i + 1) % len(polygon)
		ok, res = LinesIntersect2(polygon[i], polygon[j], a, b)
		if ok == .intersect {
			sameA :=
				(math.abs((a.x) - (res.x)) <= epsilon(T) * T(16)) &&
				(math.abs((a.y) - (res.y)) <= epsilon(T) * T(16))
			sameB :=
				(math.abs((b.x) - (res.x)) <= epsilon(T) * T(16)) &&
				(math.abs((b.y) - (res.y)) <= epsilon(T) * T(16))
			if sameA || sameB do continue
			return true
		}
	}

	return false
}

IntersectKind :: enum u8 {
	none,
	collinear,
	intersect,
}

// based on http://local.wasp.uwa.edu.au/~pbourke/geometry/lineline2d/, edgeA => "line a", edgeB => "line b"
LinesIntersect2 :: proc "contextless" (
	a1: [2]$T,
	a2: [2]T,
	b1: [2]T,
	b2: [2]T,
	checkIsTouching: bool = false,
) -> (
	IntersectKind,
	[2]T,
) where intrinsics.type_is_float(T) {
	if checkIsTouching {
		sameAB1 := a1.x == b1.x && a1.y == b1.y
		sameAB2 := a1.x == b2.x && a1.y == b2.y
		sameA2B1 := a2.x == b1.x && a2.y == b1.y
		sameA2B2 := a2.x == b2.x && a2.y == b2.y
		if sameAB1 || sameAB2 do return .none, a1
		else if sameA2B1 || sameA2B2 do return .none, a2
	}

	zero := T(0)
	den := (b2.y - b1.y) * (a2.x - a1.x) - (b2.x - b1.x) * (a2.y - a1.y)
	if den == zero {
		return .collinear, {}
	}

	ua := (b2.x - b1.x) * (a1.y - b1.y) - (b2.y - b1.y) * (a1.x - b1.x)
	ub := (a2.x - a1.x) * (a1.y - b1.y) - (a2.y - a1.y) * (a1.x - b1.x)

	resDen :=
		den > zero ? (ua >= zero && ub >= zero && ua <= den && ub <= den) : (ua >= den && ub >= den && ua <= zero && ub <= zero)

	t := ua / den
	return resDen ? .intersect : .none, [2]T{a1.x + t * (a2.x - a1.x), a1.y + t * (a2.y - a1.y)}
}

LinesIntersect3 :: proc "contextless" (
	a1: [2]$T,
	a2: [2]T,
	b1: [2]T,
	b2: [2]T,
	checkIsTouching: bool = false,
) -> IntersectKind where intrinsics.type_is_float(T) {
	sameA1B1 := a1.x == b1.x && a1.y == b1.y
	sameA2B1 := a2.x == b1.x && a2.y == b1.y
	sameA1B2 := a1.x == b2.x && a1.y == b2.y
	sameA2B2 := a2.x == b2.x && a2.y == b2.y
	if checkIsTouching && (sameA1B1 || sameA2B1 || sameA1B2 || sameA2B2) do return .none

	zero := T(0)
	den := (b2.y - b1.y) * (a2.x - a1.x) - (b2.x - b1.x) * (a2.y - a1.y)
	if den == zero do return .collinear
	ua := (b2.x - b1.x) * (a1.y - b1.y) - (b2.y - b1.y) * (a1.x - b1.x)
	ub := (a2.x - a1.x) * (a1.y - b1.y) - (a2.y - a1.y) * (a1.x - b1.x)
	resDen :=
		den > zero ? (ua >= zero && ub >= zero && ua <= den && ub <= den) : (ua >= den && ub >= den && ua <= zero && ub <= zero)
	return resDen ? .intersect : .none
}

NearestPointBetweenPointAndLine :: proc "contextless" (
	p: [2]$T,
	l0: [2]T,
	l1: [2]T,
) -> [2]T where intrinsics.type_is_float(T) {
	AB := l1 - l0
	AC := p - l0

	return l0 + AB * (linalg.vector_dot(AB, AC) / linalg.vector_dot(AB, AB))
}

Circle :: struct(T: typeid) where intrinsics.type_is_float(T) {
	p:          [2]T,
	rectRadius: T,
}

Circlef32 :: Circle(f32)
Circlef64 :: Circle(f64)

PolyOrientation :: enum {
	Clockwise,
	CounterClockwise,
}

OppPolyOrientation :: #force_inline proc "contextless" (ccw: PolyOrientation) -> PolyOrientation {
	return ccw == .Clockwise ? .CounterClockwise : .Clockwise
}

xyMirrorPoint :: #force_inline proc "contextless" (
	pivot: [2]$T,
	target: [2]T,
) -> [2]T where intrinsics.type_is_float(T) {
	return [2]T{2.0, 2.0} * pivot - target
}

Area :: union($T: typeid) where intrinsics.type_is_numeric(T) {
	Rect_(T),
	[][2]T,
}

Areaf32 :: Area(f32)
Areaf64 :: Area(f64)
Areai32 :: Area(i32)

AreaPointIn :: #force_inline proc "contextless" (
	area: Area($T),
	pt: [2]T,
) -> bool where intrinsics.type_is_numeric(T) {
	switch a in area {
	case Rect_(T):
		return RectPointIn(a, pt)
	case [][2]T:
		return PointInPolygon(pt, a) != .Outside
	}
	return false
}


// precondition: l1 <> l2
// Returns the shortest squared distance from pt to the segment [l1, l2].
@(require_results)
ShortestLength2Line :: proc "contextless" (
	pt, l1, l2: [2]$T,
) -> T where intrinsics.type_is_float(T) {
	zero := T(0)

	dx := ((l2.x) - (l1.x))
	dy := ((l2.y) - (l1.y))
	ax := ((pt.x) - (l1.x))
	ay := ((pt.y) - (l1.y))
	qNum := ((((ax) * (dx))) + (((ay) * (dy))))
	denom := ((((dx) * (dx))) + (((dy) * (dy))))
	if ((qNum) < (zero)) {
		return (((ax) * (ax))) + (((ay) * (ay)))
	} else if ((qNum) > (denom)) {
		bx := ((pt.x) - (l2.x))
		by := ((pt.y) - (l2.y))
		return (((bx) * (bx))) + (((by) * (by)))
	} else {
		cross := ((((ax) * (dy))) - (((dx) * (ay))))
		return ((cross) * (cross)) / denom
	}
}


@(require_results)
srtc2dMatrix :: proc "contextless" (
	t: linalg.Vector3f32,
	s: linalg.Vector2f32,
	r: f32,
	cp: linalg.Vector2f32,
) -> linalg.Matrix4x4f32 {
	pivot := linalg.matrix4_translate(linalg.Vector3f32{cp.x, cp.y, 0.0})
	translation := linalg.matrix4_translate(t)
	rotation := linalg.matrix4_rotate_f32(r, linalg.Vector3f32{0.0, 0.0, 1.0})
	scale := linalg.matrix4_scale(linalg.Vector3f32{s.x, s.y, 1.0})
	return linalg.mul(translation, linalg.mul(rotation, linalg.mul(scale, pivot)))
}

@(require_results)
srt2dMatrix :: proc "contextless" (
	t: linalg.Vector3f32,
	s: linalg.Vector2f32,
	r: f32,
) -> linalg.Matrix4x4f32 {
	translation := linalg.matrix4_translate(t)
	rotation := linalg.matrix4_rotate_f32(r, linalg.Vector3f32{0.0, 0.0, 1.0})
	scale := linalg.matrix4_scale(linalg.Vector3f32{s.x, s.y, 1.0})
	return linalg.mul(translation, linalg.mul(rotation, scale))
}

@(require_results)
st2dMatrix :: proc "contextless" (
	t: linalg.Vector3f32,
	s: linalg.Vector2f32,
) -> linalg.Matrix4x4f32 {
	translation := linalg.matrix4_translate(t)
	scale := linalg.matrix4_scale(linalg.Vector3f32{s.x, s.y, 1.0})
	return linalg.mul(translation, scale)
}

@(require_results)
rt2dMatrix :: proc "contextless" (t: linalg.Vector3f32, r: f32) -> linalg.Matrix4x4f32 {
	translation := linalg.matrix4_translate(t)
	rotation := linalg.matrix4_rotate_f32(r, linalg.Vector3f32{0.0, 0.0, 1.0})
	return linalg.mul(translation, rotation)
}


@(require_results)
t2dMatrix :: proc "contextless" (t: linalg.Vector3f32) -> linalg.Matrix4x4f32 {
	translation := linalg.matrix4_translate(t)
	return translation
}

@(require_results)
src2dMatrix :: proc "contextless" (
	s: linalg.Vector2f32,
	r: f32,
	cp: linalg.Vector2f32,
) -> linalg.Matrix4x4f32 {
	pivot := linalg.matrix4_translate(linalg.Vector3f32{cp.x, cp.y, 0.0})
	rotation := linalg.matrix4_rotate_f32(r, linalg.Vector3f32{0.0, 0.0, 1.0})
	scale := linalg.matrix4_scale(linalg.Vector3f32{s.x, s.y, 1.0})
	return linalg.mul(rotation, linalg.mul(scale, pivot))
}

@(require_results)
sr2dMatrix :: proc "contextless" (s: linalg.Vector2f32, r: f32) -> linalg.Matrix4x4f32 {
	rotation := linalg.matrix4_rotate_f32(r, linalg.Vector3f32{0.0, 0.0, 1.0})
	scale := linalg.matrix4_scale(linalg.Vector3f32{s.x, s.y, 1.0})
	return linalg.mul(rotation, scale)
}

@(require_results)
s2dMatrix :: proc "contextless" (s: linalg.Vector2f32) -> linalg.Matrix4x4f32 {
	scale := linalg.matrix4_scale(linalg.Vector3f32{s.x, s.y, 1.0})
	return scale
}

@(require_results)
r2dMatrix :: proc "contextless" (r: f32) -> linalg.Matrix4x4f32 {
	rotation := linalg.matrix4_rotate_f32(r, linalg.Vector3f32{0.0, 0.0, 1.0})
	return rotation
}

@(require_results)
srt2dMatrix2 :: proc "contextless" (
	t: linalg.Vector3f32,
	s: linalg.Vector2f32,
	r: f32,
	cp: linalg.Vector2f32,
) -> linalg.Matrix4x4f32 {
	if cp != {0.0, 0.0} {
		return srtc2dMatrix(t, s, r, cp)
	}
	if r != 0.0 {
		if s != {1.0, 1.0} {
			return srt2dMatrix(t, s, r)
		} else {
			return rt2dMatrix(t, r)
		}
	}
	if s != {1.0, 1.0} {
		return st2dMatrix(t, s)
	}
	return t2dMatrix(t)
}

@(require_results)
sr2dMatrix2 :: proc "contextless" (
	s: linalg.Vector2f32,
	r: f32,
	cp: linalg.Vector2f32,
) -> Maybe(linalg.Matrix4x4f32) {
	if cp != {0.0, 0.0} {
		return src2dMatrix(s, r, cp)
	}
	if r != 0.0 {
		if s != {1.0, 1.0} {
			return sr2dMatrix(s, r)
		} else {
			return r2dMatrix(r)
		}
	}
	if s != {1.0, 1.0} {
		return s2dMatrix(s)
	}
	return nil
}


@(require_results)
CvtQuadraticToCubic0 :: #force_inline proc "contextless" (
	_start: [2]$T,
	_control: [2]T,
) -> [2]T where intrinsics.type_is_float(T) {
	return [2]T {
		_start[0] + T(2.0 / 3.0) * (_control[0] - _start[0]),
		_start[1] + T(2.0 / 3.0) * (_control[1] - _start[1]),
	}
}
CvtQuadraticToCubic1 :: #force_inline proc "contextless" (
	_end: [2]$T,
	_control: [2]T,
) -> [2]T where intrinsics.type_is_float(T) {
	return CvtQuadraticToCubic0(_end, _control)
}

//(p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x) == (p2.x - p1.x) * (p3.y - p2.y) - (p2.y - p1.y) * (p3.x - p2.x)
@(require_results)
CrossProductSign :: proc "contextless" (
	p1, p2, p3: [2]$T,
) -> int where intrinsics.type_is_float(T) {
	a := ((p2.x) - (p1.x))
	b := ((p3.y) - (p2.y))
	c := ((p2.y) - (p1.y))
	d := ((p3.x) - (p2.x))
	ab := ((a) * (b))
	cd := ((c) * (d))
	if ((ab) > (cd)) do return 1
	if ((ab) < (cd)) do return -1
	return 0
}

DotProduct :: proc "contextless" (p1, p2, p3: [2]$T) -> T where intrinsics.type_is_float(T) {
	a := ((p2.x) - (p1.x))
	b := ((p3.x) - (p2.x))
	c := ((p2.y) - (p1.y))
	d := ((p3.y) - (p2.y))
	return (((a) * (b))) + (((c) * (d)))
}

// InCircleTest: returns determinant. Positive => D inside circle through A,B,C (when ABC is CCW).
InCircleTest :: proc "contextless" (
	ptA, ptB, ptC, ptD: [2]$T,
) -> T where intrinsics.type_is_float(T) {
	m00 := ((ptA.x) - (ptD.x))
	m01 := ((ptA.y) - (ptD.y))
	m02 := ((((m00) * (m00))) + (((m01) * (m01))))
	m10 := ((ptB.x) - (ptD.x))
	m11 := ((ptB.y) - (ptD.y))
	m12 := ((((m10) * (m10))) + (((m11) * (m11))))
	m20 := ((ptC.x) - (ptD.x))
	m21 := ((ptC.y) - (ptD.y))
	m22 := ((((m20) * (m20))) + (((m21) * (m21))))
	return(
		(((((m00) * (((((m11) * (m22))) - (((m21) * (m12))))))) -
				(((m10) * (((((m01) * (m22))) - (((m21) * (m02))))))))) +
		(((m20) * (((((m01) * (m12))) - (((m11) * (m02))))))) \
	)
}

GetAngle :: proc(a, b, c: [2]$T) -> T where intrinsics.type_is_float(T) {
	//https://stackoverflow.com/a/3487062/359538
	abx := b.x - a.x
	aby := b.y - a.y
	bcx := b.x - c.x
	bcy := b.y - c.y
	dp := abx * bcx + aby * bcy
	cp := abx * bcy - aby * bcx
	return math.atan2(cp, dp) //range between -Pi and Pi
}

// Evaluate any Bezier segment (line, quad, cubic) at parameter t.
evalBezierSegment :: proc "contextless" (
	kind: BezierKind,
	pts: [4][2]$T,
	t: T,
) -> [2]T where intrinsics.type_is_float(T) {
	switch kind {
	case .Line:
		u := 1 - t
		return {u * pts[0].x + t * pts[1].x, u * pts[0].y + t * pts[1].y}
	case .Quad:
		return EvalBezier(kind, pts, t)
	case .Cubic:
		return EvalBezier(kind, pts, t)
	}
	return {}
}
