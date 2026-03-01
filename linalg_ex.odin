package geometry

import "base:intrinsics"
import "core:math"
import "core:mem"
import "core:math/fixed"
import "core:math/linalg"

Recti32 :: Rect_(i32)
Rectu32 :: Rect_(u32)
Rectf32 :: Rect_(f32)

center_pt_pos :: enum {
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

Rect_ :: struct($T: typeid) where intrinsics.type_is_numeric(T) {
	left: T,
	right: T,
	top: T,
	bottom: T,
}


Rect_Init :: #force_inline proc "contextless"(left: $T, right: T, top: T, bottom: T) -> Rect_(T) {
	res: Rect_(T)
	res.left = left
	res.right = right
	res.top = top
	res.bottom = bottom
	return res
}

Check_Rect :: #force_inline proc "contextless" (_pts:[4][$N]$T) -> bool where N >= 2 && intrinsics.type_is_numeric(T) {
	return !(_pts[0].y != _pts[1].y || _pts[2].y != _pts[3].y || _pts[0].x != _pts[2].x || _pts[1].x != _pts[3].x)
}

Rect_MulMatrix :: proc "contextless" (_r: Rectf32, _mat: linalg.Matrix4x4f32) -> (Rectf32, bool) {
	tps := __Rect_MulMatrix(_r, _mat)

	if Check_Rect(tps) do return {}, false

	return Rectf32{
		left = tps[0].x,
		right = tps[3].x,
		top = tps[0].y,
		bottom = tps[3].y,
	}, true
}
Rect_DivMatrix :: proc "contextless" (_r: Rectf32, _mat: linalg.Matrix4x4f32) -> (Rectf32, bool) {
	// Apply inverse transform (Rectf32 / M == Rectf32 * inverse(M))
	return Rect_MulMatrix(_r, linalg.inverse(_mat))
}

@private __Rect_MulMatrix :: proc "contextless" (_r: Rectf32, _mat: linalg.Matrix4x4f32) -> [4]linalg.Vector4f32 {
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

	if t0.w != 0 { tx0 /= t0.w; ty0 /= t0.w }
	if t1.w != 0 { tx1 /= t1.w; ty1 /= t1.w }
	if t2.w != 0 { tx2 /= t2.w; ty2 /= t2.w }
	if t3.w != 0 { tx3 /= t3.w; ty3 /= t3.w }

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
Area_MulMatrix :: proc (_a: Areaf32, _mat: linalg.Matrix4x4f32, allocator := context.allocator) -> Areaf32 {
	switch &n in _a {
	case Rectf32:
		res := __Rect_MulMatrix(n, _mat)
		if Check_Rect(res) {
			min_x := min(res[0].x, res[3].x)
			max_x := max(res[0].x, res[3].x)
			min_y := min(res[0].y, res[3].y)
			max_y := max(res[0].y, res[3].y)
			return Rectf32{
				left = min_x,
				right = max_x,
				top = max_y,
				bottom = min_y,
			}
		}
		res2 := make_non_zeroed_slice([][2]f32, 4, allocator)
		res2[0] = res[0].xy
		res2[1] = res[1].xy
		res2[2] = res[3].xy
		res2[3] = res[2].xy
		return res2
	case [][2]f32:
		return __Poly_MulMatrix(n, _mat, allocator)
	case ImageArea:
		panic_contextless("ImageArea: Available only for ImageButton\n")
	}
	return {}
}

@private __Poly_MulMatrix :: proc (_p:[][$N]f32, _mat: linalg.Matrix4x4f32, allocator := context.allocator) -> Areaf32 where N >= 2 {
	res: [][2]f32 = make_non_zeroed_slice([][2]f32, len(_p), allocator)
	for i in 0..<len(res) {
		when N == 4 {
			r := linalg.mul(_mat, _p[i])
		} else when N == 2 {
			r := linalg.mul(_mat, linalg.Vector4f32{_p[i].x, _p[i].y, 0.0, 1.0})
		} else when N == 3 {
			r := linalg.mul(_mat, linalg.Vector4f32{_p[i].x, _p[i].y, _p[i].z, 1.0})
		} else {
			#panic("not implemented")
		}
		if r.w != 0 { r.x /= r.w; r.y /= r.w }
		res[i] = r.xy
	}
	return res
}
Rect_LeftTop :: #force_inline proc "contextless" (_r: Rect_($T)) -> [2]T {
	return [2]T{_r.left, _r.top}
}
Rect_RightBottom :: #force_inline proc "contextless" (_r: Rect_($T)) -> [2]T {
	return [2]T{_r.right, _r.bottom}
}
Rect_And :: #force_inline proc "contextless" (_r1: Rect_($T), _r2: Rect_(T)) -> Rect_(T) #no_bounds_check {
	res: Rect_(T)
	res.left = max(_r1.left, _r2.left)
	res.right = min(_r1.right, _r2.right)
	if res.right < res.left do return {}

	r1_top := max(_r1.top, _r1.bottom)
	r1_bottom := min(_r1.top, _r1.bottom)
	r2_top := max(_r2.top, _r2.bottom)
	r2_bottom := min(_r2.top, _r2.bottom)

	y_top := min(r1_top, r2_top)
	y_bottom := max(r1_bottom, r2_bottom)

	if _r1.top >= _r1.bottom {
		res.top = y_top
		res.bottom = y_bottom
	} else {
		res.top = y_bottom
		res.bottom = y_top
	}
	return res
}
Rect_Or :: #force_inline proc "contextless" (_r1: Rect_($T), _r2: Rect_(T)) -> Rect_(T) #no_bounds_check {
	res: Rect_(T)

	res.left = min(_r1.left, _r2.left)
	res.right = max(_r1.right, _r2.right)

	r1_top := max(_r1.top, _r1.bottom)
	r1_bottom := min(_r1.top, _r1.bottom)
	r2_top := max(_r2.top, _r2.bottom)
	r2_bottom := min(_r2.top, _r2.bottom)

	y_top := max(r1_top, r2_top)
	y_bottom := min(r1_bottom, r2_bottom)

	if _r1.top >= _r1.bottom {
		res.top = y_top
		res.bottom = y_bottom
	} else {
		res.top = y_bottom
		res.bottom = y_top
	}
	return res
}
Rect_PointIn :: #force_inline proc "contextless" (_r: Rect_($T), p: [2]T) -> bool #no_bounds_check {
	return p.x >= _r.left && p.x <= _r.right && (_r.top > _r.bottom ? (p.y <= _r.top && p.y >= _r.bottom) : (p.y >= _r.top && p.y <= _r.bottom))
}

TriangleOverlapsRect :: proc "contextless" (a, b, c: [2]$T, r: Rect_(T)) -> bool where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	if !Rect_OverlapsRect(Rect_FromPoints3(a, b, c), r) do return false
	if PointInTriangle([2]T{r.left, r.top}, a, b, c) || PointInTriangle([2]T{r.right, r.top}, a, b, c) ||
	   PointInTriangle([2]T{r.right, r.bottom}, a, b, c) || PointInTriangle([2]T{r.left, r.bottom}, a, b, c) {
		return true
	}
	if Rect_PointIn(r, a) || Rect_PointIn(r, b) || Rect_PointIn(r, c) do return true
	ok, _ := LinesIntersect2(a, b, [2]T{r.left, r.top}, [2]T{r.right, r.top})
	if ok do return true
	ok, _ = LinesIntersect2(a, b, [2]T{r.right, r.top}, [2]T{r.right, r.bottom})
	if ok do return true
	ok, _ = LinesIntersect2(a, b, [2]T{r.right, r.bottom}, [2]T{r.left, r.bottom})
	if ok do return true
	ok, _ = LinesIntersect2(a, b, [2]T{r.left, r.bottom}, [2]T{r.left, r.top})
	if ok do return true
	ok, _ = LinesIntersect2(b, c, [2]T{r.left, r.top}, [2]T{r.right, r.top})
	if ok do return true
	ok, _ = LinesIntersect2(b, c, [2]T{r.right, r.top}, [2]T{r.right, r.bottom})
	if ok do return true
	ok, _ = LinesIntersect2(b, c, [2]T{r.right, r.bottom}, [2]T{r.left, r.bottom})
	if ok do return true
	ok, _ = LinesIntersect2(b, c, [2]T{r.left, r.bottom}, [2]T{r.left, r.top})
	if ok do return true
	ok, _ = LinesIntersect2(c, a, [2]T{r.left, r.top}, [2]T{r.right, r.top})
	if ok do return true
	ok, _ = LinesIntersect2(c, a, [2]T{r.right, r.top}, [2]T{r.right, r.bottom})
	if ok do return true
	ok, _ = LinesIntersect2(c, a, [2]T{r.right, r.bottom}, [2]T{r.left, r.bottom})
	if ok do return true
	ok, _ = LinesIntersect2(c, a, [2]T{r.left, r.bottom}, [2]T{r.left, r.top})
	return ok
}

TriangleOverlapsTriangle :: proc "contextless" (a1, b1, c1, a2, b2, c2: [2]$T) -> bool where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	if !Rect_OverlapsRect(Rect_FromPoints3(a1, b1, c1), Rect_FromPoints3(a2, b2, c2)) do return false
	if PointInTriangle(a1, a2, b2, c2) || PointInTriangle(b1, a2, b2, c2) || PointInTriangle(c1, a2, b2, c2) do return true
	if PointInTriangle(a2, a1, b1, c1) || PointInTriangle(b2, a1, b1, c1) || PointInTriangle(c2, a1, b1, c1) do return true
	edges1 := [][2][2]T{{a1, b1}, {b1, c1}, {c1, a1}}
	edges2 := [][2][2]T{{a2, b2}, {b2, c2}, {c2, a2}}
	for e1 in edges1 {
		for e2 in edges2 {
			if LinesIntersect(e1[0], e1[1], e2[0], e2[1]) do return true
		}
	}
	return false
}

// Polygon-polygon overlap: vertex containment or edge intersection.
PolygonOverlapsPolygon :: proc "contextless" (poly1, poly2: [][2]$T) -> bool where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	if len(poly1) < 3 || len(poly2) < 3 do return false
	for p in poly1 {
		if PointInPolygon(p, poly2) do return true
	}
	for p in poly2 {
		if PointInPolygon(p, poly1) do return true
	}
	for i in 0..<len(poly1) {
		a1 := poly1[i]
		b1 := poly1[(i + 1) % len(poly1)]
		for j in 0..<len(poly2) {
			a2 := poly2[j]
			b2 := poly2[(j + 1) % len(poly2)]
			if LinesIntersect(a1, b1, a2, b2) do return true
		}
	}
	return false
}

Rect_Move :: #force_inline proc "contextless" (_r: Rect_($T), p: [2]T) -> Rect_(T) #no_bounds_check {
	res: Rect_(T)
	res.left = _r.left + p.x
	res.top = _r.top + p.y
	res.right = _r.right + p.x
	res.bottom = _r.bottom + p.y
	return res
}

PointInTriangle :: proc "contextless" (p : [2]$T, a : [2]T, b : [2]T, c : [2]T) -> bool where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	when intrinsics.type_is_float(T) {
		x0 := c.x - a.x
		y0 := c.y - a.y
		x1 := b.x - a.x
		y1 := b.y - a.y
		x2 := p.x - a.x
		y2 := p.y - a.y

		dot00 := x0 * x0 + y0 * y0
		dot01 := x0 * x1 + y0 * y1
		dot02 := x0 * x2 + y0 * y2
		dot11 := x1 * x1 + y1 * y1
		dot12 := x1 * x2 + y1 * y2
		denominator := dot00 * dot11 - dot01 * dot01
		if (denominator == 0.0) do return false

		inverseDenominator := 1.0 / denominator
		u := (dot11 * dot02 - dot01 * dot12) * inverseDenominator
		v := (dot00 * dot12 - dot01 * dot02) * inverseDenominator

		return (u > 0.0) && (v > 0.0) && (u + v < 1.0)
	} else {
		F :: intrinsics.type_polymorphic_record_parameter_value(T, 1)

		x0 :T = fixed.sub(c.x, a.x)
		y0 :T = fixed.sub(c.y, a.y)
		x1 :T = fixed.sub(b.x, a.x)
		y1 :T = fixed.sub(b.y, a.y)
		x2 :T = fixed.sub(p.x, a.x)
		y2 :T = fixed.sub(p.y, a.y)

		dot00 :T = fixed.add(fixed.mul(x0, x0), fixed.mul(y0, y0))
		dot01 :T = fixed.add(fixed.mul(x0, x1), fixed.mul(y0, y1))
		dot02 :T = fixed.add(fixed.mul(x0, x2), fixed.mul(y0, y2))
		dot11 :T = fixed.add(fixed.mul(x1, x1), fixed.mul(y1, y1))
		dot12 :T = fixed.add(fixed.mul(x1, x2), fixed.mul(y1, y2))
		denominator :T = fixed.sub(fixed.mul(dot00, dot11), fixed.mul(dot01, dot01))
		if (denominator.i == 0) do return false

		inverseDenominator :T = fixed.div(T{i=1<<F}, denominator)
		u :T = fixed.mul(fixed.sub(fixed.mul(dot11, dot02), fixed.mul(dot01, dot12)), inverseDenominator)
		v :T = fixed.mul(fixed.sub(fixed.mul(dot00, dot12), fixed.mul(dot01, dot02)), inverseDenominator)

		return (u.i > 0) && (v.i > 0) && (u.i + v.i < 1<<F)
	}
}

PointInLine :: proc "contextless" (p:[2]$T, l0:[2]T, l1:[2]T) -> (T, bool) where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed){
	when intrinsics.type_is_float(T) {
		A := (l0.y - l1.y) / (l0.x - l1.x)
		B := l0.y - A * l0.x

		pY := A * p.x + B
		EP :: epsilon(T) * 20// about count float operations
		res := p.y >= pY - EP && p.y <= pY + EP
		t :T = 0.0
		if res {
			minX := min(l0.x, l1.x)
			maxX := max(l0.x, l1.x)
			t = (p.x - minX) / (maxX - minX)
		}

		return res &&
			p.x >= min(l0.x, l1.x) &&
			p.x <= max(l0.x, l1.x) &&
			p.y >= min(l0.y, l1.y) &&
			p.y <= max(l0.y, l1.y), t
	} else {
		A :T = fixed.div(fixed.sub(l0.y, l1.y), fixed.sub(l0.x, l1.x))
		B :T = fixed.sub(l0.y, fixed.mul(A, l0.x))

		pY :T = fixed.add(fixed.mul(A, p.x), B)
		t :T = {}
		if p.y.i == pY.i {
			minX :T = min_fixed(l0.x, l1.x)
			maxX :T = max_fixed(l0.x, l1.x)
			t = fixed.div(fixed.sub(p.x, minX), fixed.sub(maxX, minX))
		}

		return t, p.y.i == pY.i &&
			p.x.i >= min_fixed(l0.x, l1.x).i &&
			p.x.i <= max_fixed(l0.x, l1.x).i &&
			p.y.i >= min_fixed(l0.y, l1.y).i &&
			p.y.i <= max_fixed(l0.y, l1.y).i
	}
}

min_fixed :: proc "contextless" (v0:$T, v1:T) -> T where intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	return v0.i < v1.i ? v0 : v1
}

max_fixed :: proc "contextless" (v0:$T, v1:T) -> T where intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	return v0.i > v1.i ? v0 : v1
}


SubdivLine :: proc "contextless" (pts: [2][2]$T, subdiv: T) -> (pt01: [2]T) where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	when intrinsics.type_is_float(T) {
		pt01 = linalg.lerp(pts[0], pts[1], subdiv)
	} else {
		subdiv2 := splat_2_fixed(subdiv)
		pt01 = lerp_fixed(pts[0], pts[1], subdiv2)
	}
	return
}

SubdivQuadraticBezier :: proc "contextless" (pts: [3][2]$T, subdiv: T) -> (pt01, pt12, pt012: [2]T) where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	when intrinsics.type_is_float(T) {
		pt01 = linalg.lerp(pts[0], pts[1], subdiv)
		pt12 = linalg.lerp(pts[1], pts[2], subdiv)
		pt012 = linalg.lerp(pt01, pt12, subdiv)
	} else {
		subdiv2 := splat_2_fixed(subdiv)
		pt01 = lerp_fixed(pts[0], pts[1], subdiv2)
		pt12 = lerp_fixed(pts[1], pts[2], subdiv2)
		pt012 = lerp_fixed(pt01, pt12, subdiv2)
	}
	return
}

SubdivCubicBezier :: proc "contextless" (pts: [4][2]$T, subdiv: T) -> (pt01, pt12, pt23, pt012, pt123, pt0123: [2]T) where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	when intrinsics.type_is_float(T) {
		pt01 = linalg.lerp(pts[0], pts[1], subdiv)
		pt12 = linalg.lerp(pts[1], pts[2], subdiv)
		pt23 = linalg.lerp(pts[2], pts[3], subdiv)
		pt012 = linalg.lerp(pt01, pt12, subdiv)
		pt123 = linalg.lerp(pt12, pt23, subdiv)
		pt0123 = linalg.lerp(pt012, pt123, subdiv)
	} else {
		subdiv2 := splat_2_fixed(subdiv)
		pt01 = lerp_fixed(pts[0], pts[1], subdiv2)
		pt12 = lerp_fixed(pts[1], pts[2], subdiv2)
		pt23 = lerp_fixed(pts[2], pts[3], subdiv2)
		pt012 = lerp_fixed(pt01, pt12, subdiv2)
		pt123 = lerp_fixed(pt12, pt23, subdiv2)
		pt0123 = lerp_fixed(pt012, pt123, subdiv2)
	}
	return
}

// Fixed-point version of linalg.lerp: result = a + t*(b - a). t is typically in [0, 1].
// Single proc with type_is_array branch: scalar Fixed or [N]Fixed (component-wise with scalar t).
lerp_fixed :: proc "contextless" (a, b, t: $T) -> T where intrinsics.type_is_specialization_of(intrinsics.type_elem_type(T), fixed.Fixed) #no_bounds_check {
	when intrinsics.type_is_array(T) {
		res: T
		#unroll for i in 0..<len(T) {
			res[i] = lerp_fixed(a[i], b[i], t[i])
		}
		return res
	} else {
		return fixed.add(a, fixed.mul(t, fixed.sub(b, a)))
	}
}

PointDeltaInLine :: proc "contextless" (p:[2]$T, l0:[2]T, l1:[2]T) -> T where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	when intrinsics.type_is_float(T) {
		pp := NearestPointBetweenPointAndLine(p, l0, l1)
		t := linalg.lerp(l0.x, l1.x, pp.x)
		return t
	} else {
		pp := NearestPointBetweenPointAndLine(p, l0, l1)
		t := lerp_fixed(l0.x, l1.x, pp.x)
		return t
	}
}

PointInVector :: proc "contextless" (p:[2]$T, v0:[2]T, v1:[2]T) -> (bool, T) where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	when intrinsics.type_is_float(T) {
		a := v1.y - v0.y
		b := v0.x - v1.x
		c := v1.x * v0.y + v0.x * v1.y
		res := a * p.x + b * p.y + c
		return res == 0, res
	} else {
		a := fixed.sub(v1.y, v0.y)
		b := fixed.sub(v0.x, v1.x)
		c := fixed.add(fixed.mul(v1.x, v0.y), fixed.mul(v0.x, v1.y))
		res := fixed.add(fixed.add(fixed.mul(a, p.x), fixed.mul(b, p.y)), c)
		return res.i == 0, res
	}
}

PointLineLeftOrRight :: #force_inline proc "contextless" (p : [2]$T, l0 : [2]T, l1 : [2]T) -> T where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	when intrinsics.type_is_float(T) {
		return (l1.x - l0.x) * (p.y - l0.y) - (p.x - l0.x) * (l1.y - l0.y)
	} else {
		return fixed.sub(fixed.mul(fixed.sub(l1.x, l0.x), fixed.sub(p.y, l0.y)), fixed.mul(fixed.sub(p.x, l0.x), fixed.sub(l1.y, l0.y)))
	}
}

PointInPolygon :: proc "contextless" (p: [2]$T, polygon:[][2]T) -> bool where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	when intrinsics.type_is_float(T) {
		crossProduct :: proc "contextless" (p1: [2]$T, p2 : [2]T, p3 : [2]T) -> T {
			return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)
		}
		isPointOnSegment :: proc "contextless" (p: [2]$T, p1 : [2]T, p2 : [2]T) -> bool {
			return crossProduct(p1, p2, p) == 0 && p.x >= min(p1.x, p2.x) && p.x <= max(p1.x, p2.x) && p.y >= min(p1.y, p2.y) && p.y <= max(p1.y, p2.y)
		}
		windingNumber := 0
		for i in 0..<len(polygon) {
			p1 := polygon[i]
			p2 := polygon[(i + 1) % len(polygon)]
			if isPointOnSegment(p, p1, p2) do return false
			if p1.y <= p.y {
				if p2.y > p.y && crossProduct(p1, p2, p) > 0 do windingNumber += 1
			} else {
				if p2.y <= p.y && crossProduct(p1, p2, p) < 0 do windingNumber -= 1
			}
		}
		return windingNumber != 0
	} else {
		crossProduct :: proc "contextless" (p1: [2]$T, p2 : [2]T, p3 : [2]T) -> T {
			return fixed.sub(fixed.mul(fixed.sub(p2.x, p1.x), fixed.sub(p3.y, p1.y)), fixed.mul(fixed.sub(p2.y, p1.y), fixed.sub(p3.x, p1.x)))
		}
		isPointOnSegment :: proc "contextless" (p: [2]$T, p1 : [2]T, p2 : [2]T) -> bool {
			cp := crossProduct(p1, p2, p)
			return cp.i == 0 && p.x.i >= min_fixed(p1.x, p2.x).i && p.x.i <= max_fixed(p1.x, p2.x).i && p.y.i >= min_fixed(p1.y, p2.y).i && p.y.i <= max_fixed(p1.y, p2.y).i
		}
		windingNumber := 0
		for i in 0..<len(polygon) {
			p1 := polygon[i]
			p2 := polygon[(i + 1) % len(polygon)]
			if isPointOnSegment(p, p1, p2) do return false
			if p1.y.i <= p.y.i {
				if p2.y.i > p.y.i && crossProduct(p1, p2, p).i > 0 do windingNumber += 1
			} else {
				if p2.y.i <= p.y.i && crossProduct(p1, p2, p).i < 0 do windingNumber -= 1
			}
		}
		return windingNumber != 0
	}
}

vector_cross2_fixed :: #force_inline proc "contextless" (a, b: [2]$E) -> E where intrinsics.type_is_specialization_of(E, fixed.Fixed) {
	return fixed.sub(fixed.mul(a.x, b.y), fixed.mul(a.y, b.x))
}

CenterPointInPolygon :: proc "contextless" (polygon : [][2]$T) -> [2]T where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	when intrinsics.type_is_float(T) {
		area: f32 = 0
		p: [2]T = {0, 0}
		for i in 0..<len(polygon) {
			j := (i + 1) % len(polygon)
			factor := linalg.vector_cross2(polygon[i], polygon[j])
			area += factor
			p = (polygon[i] + polygon[j]) * math.splat_2(factor) + p
		}
		area = area / 2 * 6
		p /= math.splat_2(area)
		return p
	} else {
		F :: intrinsics.type_polymorphic_record_parameter_value(T, 1)
		area: T = {}
		p: [2]T = {}
		for i in 0..<len(polygon) {
			j := (i + 1) % len(polygon)
			factor := vector_cross2_fixed(polygon[i], polygon[j])
			area = fixed.add(area, factor)
			sum_pt := [2]T{fixed.add(polygon[i].x, polygon[j].x), fixed.add(polygon[i].y, polygon[j].y)}
			p.x = fixed.add(p.x, fixed.mul(sum_pt.x, factor))
			p.y = fixed.add(p.y, fixed.mul(sum_pt.y, factor))
		}
		area = fixed.mul(area, T{i = 3 << F})
		p.x = fixed.div(p.x, area)
		p.y = fixed.div(p.y, area)
		return p
	}
}

GetPolygonOrientation :: proc "contextless" (polygon : [][2]$T) -> PolyOrientation where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	when intrinsics.type_is_float(T) {
		res : T = 0
		for i in 0..<len(polygon) {
			j := (i + 1) % len(polygon)
			factor := (polygon[j].x - polygon[i].x) * (polygon[j].y + polygon[i].y)
			res += factor
		}
		return res > 0 ? .Clockwise : .CounterClockwise
	} else {
		res :T = {}
		for i in 0..<len(polygon) {
			j := (i + 1) % len(polygon)
			factor := (polygon[j].x.i - polygon[i].x.i) * (polygon[j].y.i + polygon[i].y.i)
			res.i += factor
		}
		return res.i > 0 ? .Clockwise : .CounterClockwise
	}
}

LineInPolygon :: proc "contextless" (a : [2]$T, b : [2]T, polygon : [][2]T, checkInsideLine := true) -> bool where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	//Points a, b must all be inside the polygon so that line a, b and polygon line segments do not intersect, so b does not need to be checked.
	if checkInsideLine && PointInPolygon(a, polygon) do return true

	res : [2]T
	ok:bool
	for i in 0..<len(polygon) {
		j := (i + 1) % len(polygon)
		ok, res = LinesIntersect(polygon[i], polygon[j], a, b)
		if ok {
			when intrinsics.type_is_float(T) {
				if a == res || b == res do continue
			} else {
				if a.i == res.i || b.i == res.i do continue
			}
			return true
		}
	}
	
	return false
}

LinesIntersect :: proc "contextless" (a1 : [2]$T, a2 : [2]T, b1: [2]T, b2 : [2]T) -> bool where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	Orientation :: proc "contextless" (p1 : [2]$T, p2 : [2]T, p3: [2]T) -> int {
		when intrinsics.type_is_float(T) {
			crossProduct := (p2.y - p1.y) * (p3.x - p2.x) - (p3.y - p2.y) * (p2.x - p1.x)
			return (crossProduct < 0.0) ? -1 : ((crossProduct > 0.0) ? 1 : 0)
		} else {
			crossProduct :T = fixed.sub(fixed.mul(fixed.sub(p2.y, p1.y), fixed.sub(p3.x, p2.x)), fixed.mul(fixed.sub(p3.y, p2.y), fixed.sub(p2.x, p1.x)))
			return (crossProduct.i < 0) ? -1 : ((crossProduct.i > 0) ? 1 : 0)
		}
	}
	return (Orientation(a1, a2, b1) != Orientation(a1, a2, b2) && Orientation(b1, b2, a1) != Orientation(b1, b2, a2))
}

// based on http://local.wasp.uwa.edu.au/~pbourke/geometry/lineline2d/, edgeA => "line a", edgeB => "line b"
LinesIntersect2 :: proc "contextless" (a1 : [2]$T, a2 : [2]T, b1: [2]T, b2 : [2]T) -> (bool, bool, [2]T) where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	when intrinsics.type_is_float(T) {
		den :T = (b2.y - b1.y) * (a2.x - a1.x) - (b2.x - b1.x) * (a2.y - a1.y)
		if den == 0.0 {
			return false, false, {}
		}

		ua := ((b2.x - b1.x) * (a1.y - b1.y) - (b2.y - b1.y) * (a1.x - b1.x)) / den
		ub := ((a2.x - a1.x) * (a1.y - b1.y) - (a2.y - a1.y) * (a1.x - b1.x)) / den

		return true, ua >= 0.0 && ub >= 0.0 && ua <= 1.0 && ub <= 1.0, [2]T{a1.x + ua * (a2.x - a1.x), a1.y + ua * (a2.y - a1.y)}
	} else {
		den :T = fixed.mul(fixed.sub(b2.y, b1.y), fixed.sub(a2.x, a1.x)) - fixed.mul(fixed.sub(b2.x, b1.x), fixed.sub(a2.y, a1.y))
		if den.i == 0 {
			return false, false, {}
		}

		ua := fixed.div(fixed.sub(fixed.mul(fixed.sub(b2.x, b1.x), fixed.sub(a1.y, b1.y)), fixed.mul(fixed.sub(b2.y, b1.y), fixed.sub(a1.x, b1.x))), den)
		ub := fixed.div(fixed.sub(fixed.mul(fixed.sub(a2.x, a1.x), fixed.sub(a1.y, b1.y)), fixed.mul(fixed.sub(a2.y, a1.y), fixed.sub(a1.x, b1.x))), den)

		F :: intrinsics.type_polymorphic_record_parameter_value(T, 1)
		return true, ua.i >= 0 && ub.i >= 0 && ua.i <= 1 << F && ub.i <= 1 << F, [2]T{fixed.add(a1.x, fixed.mul(ua, fixed.sub(a2.x, a1.x))), fixed.add(a1.y, fixed.mul(ua, fixed.sub(a2.y, a1.y)))}
	}
}

NearestPointBetweenPointAndLine :: proc "contextless" (p:[2]$T, l0:[2]T, l1:[2]T) -> [2]T where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	when intrinsics.type_is_float(T) {
		AB := l1 - l0
		AC := p - l0

		return l0 + AB * (linalg.vector_dot(AB, AC) / linalg.vector_dot(AB, AB))
	} else {
		AB := fixed.sub(l1, l0)
		AC := fixed.sub(p, l0)

		return fixed.add(l0, fixed.mul(AB, fixed.div(linalg.vector_dot_fixed(AB, AC), linalg.vector_dot_fixed(AB, AB))))
	}
}

Circle :: struct(T:typeid) where intrinsics.type_is_float(T) {
	p : [2]T,
	radius : T,
}

Circlef32 :: Circle(f32)
Circlef64 :: Circle(f64)

PolyOrientation :: enum {
	Clockwise,
	CounterClockwise,
}

OppPolyOrientation :: #force_inline proc "contextless" (ccw:PolyOrientation) -> PolyOrientation {
	return ccw == .Clockwise ? .CounterClockwise : .Clockwise
}

xy_mirror_point :: #force_inline proc "contextless" (pivot : [2]$T, target : [2]T) -> [2]T where intrinsics.type_is_float(T) || intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	when intrinsics.type_is_float(T) {
		return [2]T{2.0,2.0} * pivot - target
	} else {
		F :: intrinsics.type_polymorphic_record_parameter_value(T, 1)

		return [2]T{fixed.sub(fixed.mul(T{i=2<<F}, pivot.x), target.x), fixed.sub(fixed.mul(T{i=2<<F}, pivot.y), target.y)}
	}
}

ImageArea :: struct {}

Area :: union($T: typeid) where intrinsics.type_is_numeric(T)  {
	Rect_(T),
	[][2]T,
	ImageArea,//Available only for ImageButton
}

Areaf32 :: Area(f32)
Areaf64 :: Area(f64)
Areai32 :: Area(i32)

Area_PointIn :: #force_inline proc "contextless" (area:Area($T), pt:[2]T) -> bool where intrinsics.type_is_numeric(T) {
	switch a in area {
	case Rect_(T):return Rect_PointIn(a, pt)
	case [][2]T:return PointInPolygon(pt, a)
	case ImageArea:panic_contextless("ImageArea: Available only for ImageButton\n")
	}
	return false
}


@(require_results)
srtc_2d_matrix :: proc "contextless" (t: linalg.Vector3f32, s: linalg.Vector2f32, r: f32, cp:linalg.Vector2f32) -> linalg.Matrix4x4f32 {
	pivot := linalg.matrix4_translate(linalg.Vector3f32{cp.x,cp.y,0.0})
	translation := linalg.matrix4_translate(t)
	rotation := linalg.matrix4_rotate_f32(r, linalg.Vector3f32{0.0, 0.0, 1.0})
	scale := linalg.matrix4_scale(linalg.Vector3f32{s.x,s.y,1.0})
	return linalg.mul(translation, linalg.mul(rotation, linalg.mul(scale, pivot)))
}

@(require_results)
srt_2d_matrix :: proc "contextless" (t: linalg.Vector3f32, s: linalg.Vector2f32, r: f32) -> linalg.Matrix4x4f32 {
	translation := linalg.matrix4_translate(t)
	rotation := linalg.matrix4_rotate_f32(r, linalg.Vector3f32{0.0, 0.0, 1.0})
	scale := linalg.matrix4_scale(linalg.Vector3f32{s.x,s.y,1.0})
	return linalg.mul(translation, linalg.mul(rotation, scale))
}

@(require_results)
st_2d_matrix :: proc "contextless" (t: linalg.Vector3f32, s: linalg.Vector2f32) -> linalg.Matrix4x4f32 {
	translation := linalg.matrix4_translate(t)
	scale := linalg.matrix4_scale(linalg.Vector3f32{s.x,s.y,1.0})
	return linalg.mul(translation, scale)
}

@(require_results)
rt_2d_matrix :: proc "contextless" (t: linalg.Vector3f32, r: f32) -> linalg.Matrix4x4f32 {
	translation := linalg.matrix4_translate(t)
    rotation := linalg.matrix4_rotate_f32(r, linalg.Vector3f32{0.0, 0.0, 1.0})
	return linalg.mul(translation, rotation)
}


@(require_results)
t_2d_matrix :: proc "contextless" (t: linalg.Vector3f32) -> linalg.Matrix4x4f32 {
	translation := linalg.matrix4_translate(t)
	return translation
}

@(require_results)
src_2d_matrix :: proc "contextless" (s: linalg.Vector2f32, r: f32, cp:linalg.Vector2f32) -> linalg.Matrix4x4f32 {
	pivot := linalg.matrix4_translate(linalg.Vector3f32{cp.x,cp.y,0.0})
	rotation := linalg.matrix4_rotate_f32(r, linalg.Vector3f32{0.0, 0.0, 1.0})
	scale := linalg.matrix4_scale(linalg.Vector3f32{s.x,s.y,1.0})
	return linalg.mul(rotation, linalg.mul(scale, pivot))
}

@(require_results)
sr_2d_matrix :: proc "contextless" (s: linalg.Vector2f32, r: f32) -> linalg.Matrix4x4f32 {
	rotation := linalg.matrix4_rotate_f32(r, linalg.Vector3f32{0.0, 0.0, 1.0})
	scale := linalg.matrix4_scale(linalg.Vector3f32{s.x,s.y,1.0})
	return linalg.mul(rotation, scale)
}

@(require_results)
s_2d_matrix :: proc "contextless" (s: linalg.Vector2f32) -> linalg.Matrix4x4f32 {
	scale := linalg.matrix4_scale(linalg.Vector3f32{s.x,s.y,1.0})
	return scale
}

@(require_results)
r_2d_matrix :: proc "contextless" (r: f32) -> linalg.Matrix4x4f32 {
    rotation := linalg.matrix4_rotate_f32(r, linalg.Vector3f32{0.0, 0.0, 1.0})
	return rotation
}

@(require_results)
srt_2d_matrix2 :: proc "contextless" (t: linalg.Vector3f32, s: linalg.Vector2f32, r: f32, cp:linalg.Vector2f32) -> linalg.Matrix4x4f32 {
    if cp != {0.0, 0.0} {
        return srtc_2d_matrix(t,s,r,cp)
    }
    if r != 0.0 {
        if s != {1.0, 1.0} {
            return srt_2d_matrix(t,s,r)
        } else {
            return rt_2d_matrix(t,r)
        }
    }
    if s != {1.0, 1.0} {
        return st_2d_matrix(t,s)
    }
    return t_2d_matrix(t)
}

@(require_results)
sr_2d_matrix2 :: proc "contextless" (s: linalg.Vector2f32, r: f32, cp:linalg.Vector2f32) -> Maybe(linalg.Matrix4x4f32) {
    if cp != {0.0, 0.0} {
        return src_2d_matrix(s,r,cp)
    }
    if r != 0.0 {
        if s != {1.0, 1.0} {
            return sr_2d_matrix(s,r)
        } else {
            return r_2d_matrix(r)
        }
    }
    if s != {1.0, 1.0} {
        return s_2d_matrix(s)
    }
    return nil
}


@(require_results)
splat_2_fixed :: #force_inline proc "contextless" (v: $T) -> [2]T where intrinsics.type_is_specialization_of(T, fixed.Fixed) {
	return [2]T{v, v}
}

@(require_results)
vector_dot_fixed :: proc "contextless" (a, b: $T/[$N]$E) -> (c: E) where intrinsics.type_is_specialization_of(E, fixed.Fixed) #no_bounds_check {
	#unroll for i in 0..<N {
		c = fixed.add(c, fixed.mul(a[i], b[i]))
	}
	return
}


CvtQuadraticToCubic0 :: #force_inline proc "contextless" (_start : linalg.Vector2f32, _control : linalg.Vector2f32) -> linalg.Vector2f32 {
    return linalg.Vector2f32{ _start.x + (2.0/3.0) * (_control.x - _start.x), _start.y + (2.0/3.0) * (_control.y - _start.y) }
}
CvtQuadraticToCubic1 :: #force_inline proc "contextless" (_end : linalg.Vector2f32, _control : linalg.Vector2f32) -> linalg.Vector2f32 {
    return CvtQuadraticToCubic0(_end, _control)
}

rect_line_init :: proc "contextless" (_rect: Rectf32) -> [4]linalg.Vector2f32 {
	return [4]linalg.Vector2f32{linalg.Vector2f32{_rect.left, _rect.top}, linalg.Vector2f32{_rect.left, _rect.bottom}, linalg.Vector2f32{_rect.right, _rect.bottom}, linalg.Vector2f32{_rect.right, _rect.top}}
}

round_rect_line_init :: proc "contextless" (_rect: Rectf32, _radius: f32) -> (pts:[16]linalg.Vector2f32, curve_pts_ids:[8]u32) {
	r := _radius
	// Clamp radius to fit within rect
	half_width := (_rect.right - _rect.left) * 0.5
	half_height := abs(_rect.bottom - _rect.top) * 0.5
	r = min(r, min(half_width, half_height))
	
	t: f32 = (4.0 / 3.0) * math.tan_f32(math.PI / 8.0)
	tt := t * r
	
	// Corner centers
	top_left := linalg.Vector2f32{_rect.left + r, _rect.top + r}
	top_right := linalg.Vector2f32{_rect.right - r, _rect.top + r}
	bottom_right := linalg.Vector2f32{_rect.right - r, _rect.bottom - r}
	bottom_left := linalg.Vector2f32{_rect.left + r, _rect.bottom - r}
	
	return [16]linalg.Vector2f32{
		// Top-left corner (cubic) - counter-clockwise: from top to left
		// Note: y increases upward, _rect.top is top (larger y), _rect.bottom is bottom (smaller y)
			linalg.Vector2f32{_rect.left + r, _rect.top},
			linalg.Vector2f32{_rect.left + r - tt, _rect.top},
			linalg.Vector2f32{_rect.left, _rect.top - r + tt},
		// Left line - counter-clockwise: from top to bottom (y decreases)
			linalg.Vector2f32{_rect.left, _rect.top - r},
		// Bottom-left corner (cubic) - counter-clockwise: from left to bottom
			linalg.Vector2f32{_rect.left, _rect.bottom + r},
			linalg.Vector2f32{_rect.left, _rect.bottom + r - tt},
			linalg.Vector2f32{_rect.left + r - tt, _rect.bottom},
		// Bottom line - counter-clockwise: from left to right
			linalg.Vector2f32{_rect.left + r, _rect.bottom},
		// Bottom-right corner (cubic) - counter-clockwise: from bottom to right
			linalg.Vector2f32{_rect.right - r, _rect.bottom},
			linalg.Vector2f32{_rect.right - r + tt, _rect.bottom},
			linalg.Vector2f32{_rect.right, _rect.bottom + r - tt},
		// Right line - counter-clockwise: from bottom to top (y increases)
			linalg.Vector2f32{_rect.right, _rect.bottom + r},
		// Top-right corner (cubic) - counter-clockwise: from right to top
			linalg.Vector2f32{_rect.right, _rect.top - r},
			linalg.Vector2f32{_rect.right, _rect.top - r + tt},
			linalg.Vector2f32{_rect.right - r + tt, _rect.top},
		// Top line - counter-clockwise: from right to left
			linalg.Vector2f32{_rect.right - r, _rect.top},
	}, [8]u32{1,2, 5,6, 9,10, 13,14}
}

circle_cubic_init :: proc "contextless" (_center: linalg.Vector2f32, _r: f32) -> (pts:[12]linalg.Vector2f32, curve_pts_ids:[8]u32) {
	t: f32 = (4.0 / 3.0) * math.tan_f32(math.PI / 8.0)
	tt := t * _r
	return [12]linalg.Vector2f32{
			linalg.Vector2f32{_center.x - _r, _center.y},
			linalg.Vector2f32{_center.x - _r, _center.y - tt},
			linalg.Vector2f32{_center.x - tt, _center.y - _r},

			linalg.Vector2f32{_center.x, _center.y - _r},
			linalg.Vector2f32{_center.x + tt, _center.y - _r},
			linalg.Vector2f32{_center.x + _r, _center.y - tt},

			linalg.Vector2f32{_center.x + _r, _center.y},
			linalg.Vector2f32{_center.x + _r, _center.y + tt},
			linalg.Vector2f32{_center.x + tt, _center.y + _r},

			linalg.Vector2f32{_center.x, _center.y + _r},
			linalg.Vector2f32{_center.x - tt, _center.y + _r},
			linalg.Vector2f32{_center.x - _r, _center.y + tt},
	}, [8]u32{1,2, 4,5, 7,8, 10,11}
}

ellipse_cubic_init :: proc "contextless" (_center: linalg.Vector2f32, _rxy: linalg.Vector2f32) -> (pts:[12]linalg.Vector2f32, curve_pts_ids:[8]u32) {
	t: f32 = (4.0 / 3.0) * math.tan_f32(math.PI / 8.0)
	ttx := t * _rxy.x
	tty := t * _rxy.y
	return [12]linalg.Vector2f32{
			linalg.Vector2f32{_center.x - _rxy.x, _center.y},
			linalg.Vector2f32{_center.x - _rxy.x, _center.y - tty},
			linalg.Vector2f32{_center.x - ttx, _center.y - _rxy.y},

			linalg.Vector2f32{_center.x, _center.y - _rxy.y},
			linalg.Vector2f32{_center.x + ttx, _center.y - _rxy.y},
			linalg.Vector2f32{_center.x + _rxy.x, _center.y - tty},

			linalg.Vector2f32{_center.x + _rxy.x, _center.y},
			linalg.Vector2f32{_center.x + _rxy.x, _center.y + tty},
			linalg.Vector2f32{_center.x + ttx, _center.y + _rxy.y},

			linalg.Vector2f32{_center.x, _center.y + _rxy.y},
			linalg.Vector2f32{_center.x - ttx, _center.y + _rxy.y},
			linalg.Vector2f32{_center.x - _rxy.x, _center.y + tty},
	}, [8]u32{1,2, 4,5, 7,8, 10,11}
}

poly_transform_matrix :: proc "contextless" (inout_poly: ^shapes, F: linalg.Matrix4x4f32) {
	for &node in inout_poly.nodes {
		for &pts in node.pts {
			out := linalg.mul(F, linalg.Vector4f32{pts.x, pts.y, 0, 1})
			pts = linalg.Vector2f32{out.x, out.y} / out.w
		}
	}
}

ceil_up :: proc "contextless"(num:$T, multiple:T) -> T where intrinsics.type_is_integer(T) {
	if multiple == 0 do return num

	remain := abs(num) % multiple
	if remain == 0 do return num

	if num < 0 do return -(abs(num) + multiple - remain)
	return num + multiple - remain
}

floor_up :: proc "contextless"(num:$T, multiple:T) -> T where intrinsics.type_is_integer(T) {
	if multiple == 0 do return num

	remain := abs(num) % multiple
	if remain == 0 do return num

	if num < 0 do return -(abs(num) - remain)
	return num - remain
}
min_array :: proc "contextless" (value0:$T/[$N]$E, value1:T) -> (result:T) where intrinsics.type_is_array(T) {
	#unroll for i in 0..<len(value0)  {
		m : E = value0[i]
		if m > value1[i] do m = value1[i]
		result[i] = m
	}
	return
}
max_array :: proc "contextless" (value0:$T/[$N]$E, value1:T) -> (result:T) where intrinsics.type_is_array(T) {
	#unroll for i in 0..<len(value0)  {
		m : E = value0[i]
		if m < value1[i] do m = value1[i]
		result[i] = m
	}
	return
}

epsilon :: proc "contextless" ($T: typeid) -> T where intrinsics.type_is_float(T) {
	when T == f16 || T == f16be || T == f16le do return T(math.F16_EPSILON)
	when T == f32 || T == f32be || T == f32le do return T(math.F32_EPSILON)
	return T(math.F64_EPSILON)
}

epsilon_equal :: proc "contextless" (a:$T, b:T) -> bool where intrinsics.type_is_float(T) {
	return abs(a - b) < epsilon(T)
}
