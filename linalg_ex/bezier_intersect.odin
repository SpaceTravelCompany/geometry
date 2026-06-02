package linalg_ex

import "base:intrinsics"
//import "core:fmt"
import "core:math"
import "core:math/fixed"
import "core:math/linalg"
import "core:testing"

BezierKind :: enum u8 {
	Line,
	Quad,
	Cubic,
}

MaxBezierIntersections :: 9

@(private = "file")
_rootEps :: #force_inline proc "contextless" ($T: typeid) -> T where intrinsics.type_is_float(T) {
	when T == f64 {
		return T(1e-12)
	}
	return T(1e-6)
}

@(private = "file")
_addUnitRoot :: proc "contextless" (roots: ^[4]$T, count: ^int, root: T) where intrinsics.type_is_float(T) {
	eps := _rootEps(T)
	if root < -eps || root > 1 + eps || count^ >= len(roots^) do return
	r := math.clamp(root, T(0), T(1))
	for i in 0 ..< count^ {
		if math.abs(roots^[i] - r) <= eps do return
	}
	roots^[count^] = r
	count^ += 1
}

@(private = "file")
_solveLinearUnit :: proc "contextless" (roots: ^[4]$T, count: ^int, b, c: T) where intrinsics.type_is_float(T) {
	eps := _rootEps(T)
	if math.abs(b) <= eps do return
	_addUnitRoot(roots, count, -c / b)
}

@(private = "file")
_solveQuadraticUnit :: proc "contextless" (roots: ^[4]$T, count: ^int, a, b, c: T) where intrinsics.type_is_float(T) {
	eps := _rootEps(T)
	if math.abs(a) <= eps {
		_solveLinearUnit(roots, count, b, c)
		return
	}
	disc := b * b - T(4) * a * c
	if disc < -eps do return
	if math.abs(disc) <= eps {
		_addUnitRoot(roots, count, -b / (T(2) * a))
		return
	}
	s := math.sqrt(disc)
	_addUnitRoot(roots, count, (-b - s) / (T(2) * a))
	_addUnitRoot(roots, count, (-b + s) / (T(2) * a))
}

@(private = "file")
_solveCubicUnit :: proc "contextless" (roots: ^[4]$T, count: ^int, a, b, c, d: T) where intrinsics.type_is_float(T) {
	eps := _rootEps(T)
	if math.abs(a) <= eps {
		_solveQuadraticUnit(roots, count, b, c, d)
		return
	}

	aa := b / a
	bb := c / a
	cc := d / a
	p := bb - aa * aa / T(3)
	q := T(2) * aa * aa * aa / T(27) - aa * bb / T(3) + cc
	disc := q * q / T(4) + p * p * p / T(27)
	shift := aa / T(3)

	if disc > eps {
		s := math.sqrt(disc)
		u := math.cbrt(-q / T(2) + s)
		v := math.cbrt(-q / T(2) - s)
		_addUnitRoot(roots, count, u + v - shift)
		return
	}

	if math.abs(disc) <= eps {
		u := math.cbrt(-q / T(2))
		_addUnitRoot(roots, count, T(2) * u - shift)
		_addUnitRoot(roots, count, -u - shift)
		return
	}

	if p >= 0 do return
	r := T(2) * math.sqrt(-p / T(3))
	arg := (T(3) * q / (T(2) * p)) * math.sqrt(-T(3) / p)
	arg = math.clamp(arg, T(-1), T(1))
	theta := math.acos(arg) / T(3)
	tau := T(math.TAU)
	for k in 0 ..< 3 {
		_addUnitRoot(roots, count, r * math.cos(theta - tau * T(k) / T(3)) - shift)
	}
}

@(private = "file")
_evalAnyBezier :: proc "contextless" (
	kind: BezierKind,
	pts: [4][2]$T,
	t: T,
) -> [2]T where intrinsics.type_is_float(T) {
	if kind == .Line {
		u := T(1) - t
		return {pts[0].x * u + pts[1].x * t, pts[0].y * u + pts[1].y * t}
	}
	return EvalBezier(kind, pts, t)
}

@(private = "file")
_lineTForPoint :: proc "contextless" (line: [4][2]$T, p: [2]T) -> T where intrinsics.type_is_float(T) {
	dx := line[1].x - line[0].x
	dy := line[1].y - line[0].y
	den := dx * dx + dy * dy
	if den <= _rootEps(T) do return T(0)
	return ((p.x - line[0].x) * dx + (p.y - line[0].y) * dy) / den
}

@(private = "file")
_addLineCurveHit :: proc "contextless" (
	line: [4][2]$T,
	curveKind: BezierKind,
	curve: [4][2]T,
	tCurve: T,
	count: ^int,
	ips: ^[MaxBezierIntersections][2]T,
	tLineOut: ^[MaxBezierIntersections]T,
	tCurveOut: ^[MaxBezierIntersections]T,
) where intrinsics.type_is_float(T) {
	if count^ >= MaxBezierIntersections do return
	eps := _rootEps(T)
	p := _evalAnyBezier(curveKind, curve, math.clamp(tCurve, T(0), T(1)))
	tLine := _lineTForPoint(line, p)
	if tLine < -eps || tLine > 1 + eps do return
	tLine = math.clamp(tLine, T(0), T(1))
	tCurveClamped := math.clamp(tCurve, T(0), T(1))
	for i in 0 ..< count^ {
		if math.abs(tLineOut^[i] - tLine) <= eps && math.abs(tCurveOut^[i] - tCurveClamped) <= eps do return
	}
	ips^[count^] = p
	tLineOut^[count^] = tLine
	tCurveOut^[count^] = tCurveClamped
	count^ += 1
}

@(private = "file")
_lineCurveIntersections :: proc "contextless" (
	line: [4][2]$T,
	curveKind: BezierKind,
	curve: [4][2]T,
) -> (
	count: int,
	ips: [MaxBezierIntersections][2]T,
	tLineOut: [MaxBezierIntersections]T,
	tCurveOut: [MaxBezierIntersections]T,
) where intrinsics.type_is_float(T) {
	dx := line[1].x - line[0].x
	dy := line[1].y - line[0].y
	if math.abs(dx) <= _rootEps(T) && math.abs(dy) <= _rootEps(T) do return

	dist: [4]T
	n := bezierOrder(curveKind) - 1
	for i in 0 ..= n {
		dist[i] = (curve[i].x - line[0].x) * dy - (curve[i].y - line[0].y) * dx
	}

	roots: [4]T
	rootCount := 0
	switch curveKind {
	case .Line:
		_solveLinearUnit(&roots, &rootCount, dist[1] - dist[0], dist[0])
	case .Quad:
		a := dist[0] - T(2) * dist[1] + dist[2]
		b := T(2) * (dist[1] - dist[0])
		c := dist[0]
		_solveQuadraticUnit(&roots, &rootCount, a, b, c)
	case .Cubic:
		a := -dist[0] + T(3) * dist[1] - T(3) * dist[2] + dist[3]
		b := T(3) * dist[0] - T(6) * dist[1] + T(3) * dist[2]
		c := -T(3) * dist[0] + T(3) * dist[1]
		d := dist[0]
		_solveCubicUnit(&roots, &rootCount, a, b, c, d)
	}

	for i in 0 ..< rootCount {
		_addLineCurveHit(line, curveKind, curve, roots[i], &count, &ips, &tLineOut, &tCurveOut)
	}
	return
}

bezierOrder :: #force_inline proc "contextless" (kind: BezierKind) -> int {
	switch kind {
	case .Line:
		return 2
	case .Quad:
		return 3
	case .Cubic:
		return 4
	}
	return 0
}

// Split Bézier at t=0.5, returning left and right halves.
@(private = "file")
_splitHalf :: proc "contextless" (
	kind: BezierKind,
	pts: [4][2]$T,
) -> (
	left, right: [4][2]T,
) where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	half :=
		T(0.5) when intrinsics.type_is_float(T) else T {
			i = 1 << intrinsics.type_polymorphic_record_parameter_value(T, 1) >> 1,
		}
	switch kind {
	case .Line:
		mid := SubdivLine([2][2]T{pts[0], pts[1]}, half)
		left = {pts[0], mid, {}, {}}
		right = {mid, pts[1], {}, {}}
	case .Quad:
		p01, m, p12 := SubdivQuadraticBezier([3][2]T{pts[0], pts[1], pts[2]}, half)
		left = {pts[0], p01, m, {}}
		right = {m, p12, pts[2], {}}
	case .Cubic:
		c0, c1, m, d0, d1 := SubdivCubicBezier(pts, half)
		left = {pts[0], c0, c1, m}
		right = {m, d0, d1, pts[3]}
	}
	return
}


// Signed distance from point to line (unnormalized cross product).
@(private = "file")
_distToBaseline :: #force_inline proc "contextless" (
	px, py, ax, ay, dx, dy: $T,
) -> T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	return NumSub(NumMul(NumSub(px, ax), dy), NumMul(NumSub(py, ay), dx))
}

// Clip convex hull of distance control polygon against fat line band [d_min, d_max].
// Returns valid parameter range. t_lo > t_hi means no intersection.
@(private = "file")
_clipHull :: proc "contextless" (
	d: [4]$T,
	n: int,
	dMin, dMax: T,
) -> (
	tLo, tHi: T,
) where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	one :=
		T(1) when intrinsics.type_is_float(T) else T {
			i = 1 << intrinsics.type_polymorphic_record_parameter_value(T, 1),
		}
	zero := T(0) when intrinsics.type_is_float(T) else T{i = 0}
	tLo = one
	tHi = zero

	for i in 0 ..= n {
		if NumGe(d[i], dMin) && NumLe(d[i], dMax) {
			ti := NumRatio(i, n, T)
			if NumLt(ti, tLo) do tLo = ti
			if NumGt(ti, tHi) do tHi = ti
		}
	}

	for i in 0 ..= n {
		for j in i + 1 ..= n {
			ti := NumRatio(i, n, T)
			tj := NumRatio(j, n, T)
			dd := NumSub(d[j], d[i])
			if NumEq(dd, zero) do continue
			dt := NumSub(tj, ti)

			if NumLt(d[i], dMin) != NumLt(d[j], dMin) {
				tc := NumAdd(ti, NumMul(NumDiv(NumSub(dMin, d[i]), dd), dt))
				if NumGe(tc, zero) && NumLe(tc, one) {
					if NumLt(tc, tLo) do tLo = tc
					if NumGt(tc, tHi) do tHi = tc
				}
			}
			if NumGt(d[i], dMax) != NumGt(d[j], dMax) {
				tc := NumAdd(ti, NumMul(NumDiv(NumSub(dMax, d[i]), dd), dt))
				if NumGe(tc, zero) && NumLe(tc, one) {
					if NumLt(tc, tLo) do tLo = tc
					if NumGt(tc, tHi) do tHi = tc
				}
			}
		}
	}
	return
}

@(private = "file")
_BezClipWork :: struct($T: typeid) {
	a:       [4][2]T,
	b:       [4][2]T,
	kindA:  BezierKind,
	kindB:  BezierKind,
	aLo:    T,
	aHi:    T,
	bLo:    T,
	bHi:    T,
	swapped: bool, // true when a/b correspond to original b/a
}

// Extract sub-curve for parameter range [t_lo, t_hi].
@(private = "file")
_extractSub :: proc "contextless" (
	kind: BezierKind,
	pts: [4][2]$T,
	tLo, tHi: T,
) -> [4][2]T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	one :=
		T(1) when intrinsics.type_is_float(T) else T {
			i = 1 << intrinsics.type_polymorphic_record_parameter_value(T, 1),
		}
	zero := T(0) when intrinsics.type_is_float(T) else T{i = 0}
	result := pts

	if NumGt(tLo, zero) {
		switch kind {
		case .Line:
			p := SubdivLine([2][2]T{pts[0], pts[1]}, tLo)
			result = {p, pts[1], {}, {}}
		case .Quad:
			_, m, p12 := SubdivQuadraticBezier([3][2]T{pts[0], pts[1], pts[2]}, tLo)
			result = {m, p12, pts[2], {}}
		case .Cubic:
			_, _, m, d0, d1 := SubdivCubicBezier(pts, tLo)
			result = {m, d0, d1, pts[3]}
		}
	}

	denom := NumSub(one, tLo)
	if NumLe(denom, zero) do return result
	tAdj := NumDiv(NumSub(tHi, tLo), denom)
	if NumGe(tAdj, one) do return result

	switch kind {
	case .Line:
		p := SubdivLine([2][2]T{result[0], result[1]}, tAdj)
		result[1] = p
	case .Quad:
		p01, m, _ := SubdivQuadraticBezier([3][2]T{result[0], result[1], result[2]}, tAdj)
		result = {result[0], p01, m, {}}
	case .Cubic:
		c0, c1, m, _, _ := SubdivCubicBezier(result, tAdj)
		result = {result[0], c0, c1, m}
	}
	return result
}

// Bézier Clipping intersection (Sederberg-Nishita).
// Returns all intersections found (up to fixed capacity), with each input curve's t.
GetBezierIntersectPt :: proc "contextless" (
	kindA: BezierKind,
	aIn: [4][2]$T,
	kindB: BezierKind,
	bIn: [4][2]T,
) -> (
	count: int,
	ips: [MaxBezierIntersections][2]T,
	tAs: [MaxBezierIntersections]T,
	tBs: [MaxBezierIntersections]T,
) where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	when intrinsics.type_is_float(T) {
		if kindA == .Line {
			return _lineCurveIntersections(aIn, kindB, bIn)
		}
		if kindB == .Line {
			count, ips, tLine, tCurve := _lineCurveIntersections(bIn, kindA, aIn)
			return count, ips, tCurve, tLine
		}
	}

	one :=
		T(1) when intrinsics.type_is_float(T) else T {
			i = 1 << intrinsics.type_polymorphic_record_parameter_value(T, 1),
		}
	zero := T(0) when intrinsics.type_is_float(T) else T{i = 0}
	eps: T = epsilon(T) when intrinsics.type_is_float(T) else T{}
	epsHit := NumMul(eps, NumConst(10, T)) when intrinsics.type_is_float(T) else zero
	fourFifth := NumRatio(4, 5, T)

	Work :: _BezClipWork(T)
	stack: [64]Work
	sp := 1
	stack[0] = Work {
		a       = aIn,
		b       = bIn,
		kindA  = kindA,
		kindB  = kindB,
		aLo    = zero,
		aHi    = one,
		bLo    = zero,
		bHi    = one,
		swapped = false,
	}

	for sp > 0 {
		sp -= 1
		w := stack[sp]

		aRange := NumSub(w.aHi, w.aLo)
		bRange := NumSub(w.bHi, w.bLo)

		if NumLe(aRange, eps) && NumLe(bRange, eps) {
			na := bezierOrder(w.kindA) - 1
			ip := [2]T{NumMid(w.a[0].x, w.a[na].x), NumMid(w.a[0].y, w.a[na].y)}
			tA: T
			tB: T
			if !w.swapped {
				tA = NumMid(w.aLo, w.aHi)
				tB = NumMid(w.bLo, w.bHi)
			} else {
				tA = NumMid(w.bLo, w.bHi)
				tB = NumMid(w.aLo, w.aHi)
			}

			isDup := false
			for i in 0 ..< count {
				if NumLe(NumAbs(NumSub(tAs[i], tA)), epsHit) &&
				   NumLe(NumAbs(NumSub(tBs[i], tB)), epsHit) {
					isDup = true
					break
				}
			}
			if !isDup && count < len(ips) {
				ips[count] = ip
				tAs[count] = tA
				tBs[count] = tB
				count += 1
			}
			continue
		}

		na := bezierOrder(w.kindA) - 1
		ax := w.a[0].x
		ay := w.a[0].y
		dx := NumSub(w.a[na].x, ax)
		dy := NumSub(w.a[na].y, ay)

		if NumEq(dx, zero) && NumEq(dy, zero) {
			continue
		}

		dMin := zero
		dMax := zero
		for i in 1 ..< na {
			d := _distToBaseline(w.a[i].x, w.a[i].y, ax, ay, dx, dy)
			if NumLt(d, dMin) do dMin = d
			if NumGt(d, dMax) do dMax = d
		}

		nb := bezierOrder(w.kindB) - 1
		db: [4]T
		for i in 0 ..= nb {
			db[i] = _distToBaseline(w.b[i].x, w.b[i].y, ax, ay, dx, dy)
		}

		clipLo, clipHi := _clipHull(db, nb, dMin, dMax)
		if NumGt(clipLo, clipHi) do continue

		if NumLt(clipLo, zero) do clipLo = zero
		if NumGt(clipHi, one) do clipHi = one

		bSpan := NumSub(w.bHi, w.bLo)
		newBLo := NumAdd(w.bLo, NumMul(clipLo, bSpan))
		newBHi := NumAdd(w.bLo, NumMul(clipHi, bSpan))
		bClipped := _extractSub(w.kindB, w.b, clipLo, clipHi)

		clipRange := NumSub(clipHi, clipLo)

		if NumGt(clipRange, fourFifth) {
			if sp + 2 > len(stack) do continue
			if NumGe(aRange, bRange) {
				aLeft, aRight := _splitHalf(w.kindA, w.a)
				aMid := NumMid(w.aLo, w.aHi)
				stack[sp] = Work {
					a       = bClipped,
					b       = aLeft,
					kindA  = w.kindB,
					kindB  = w.kindA,
					aLo    = newBLo,
					aHi    = newBHi,
					bLo    = w.aLo,
					bHi    = aMid,
					swapped = !w.swapped,
				}
				sp += 1
				stack[sp] = Work {
					a       = bClipped,
					b       = aRight,
					kindA  = w.kindB,
					kindB  = w.kindA,
					aLo    = newBLo,
					aHi    = newBHi,
					bLo    = aMid,
					bHi    = w.aHi,
					swapped = !w.swapped,
				}
				sp += 1
			} else {
				bLeft, bRight := _splitHalf(w.kindB, bClipped)
				bMid := NumMid(newBLo, newBHi)
				stack[sp] = Work {
					a       = bLeft,
					b       = w.a,
					kindA  = w.kindB,
					kindB  = w.kindA,
					aLo    = newBLo,
					aHi    = bMid,
					bLo    = w.aLo,
					bHi    = w.aHi,
					swapped = !w.swapped,
				}
				sp += 1
				stack[sp] = Work {
					a       = bRight,
					b       = w.a,
					kindA  = w.kindB,
					kindB  = w.kindA,
					aLo    = bMid,
					aHi    = newBHi,
					bLo    = w.aLo,
					bHi    = w.aHi,
					swapped = !w.swapped,
				}
				sp += 1
			}
		} else {
			if sp + 1 > len(stack) do continue
			stack[sp] = Work {
				a       = bClipped,
				b       = w.a,
				kindA  = w.kindB,
				kindB  = w.kindA,
				aLo    = newBLo,
				aHi    = newBHi,
				bLo    = w.aLo,
				bHi    = w.aHi,
				swapped = !w.swapped,
			}
			sp += 1
		}
	}
	return count, ips, tAs, tBs
}

// Finds parameter t on a Bezier curve from a point by binary search.
GetBezierTFromPoint :: proc "contextless" (
	kind: BezierKind,
	pts: [4][2]$T,
	point: [2]T,
) -> (
	t: T,
	ok: bool,
) where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	zero := T(0) when intrinsics.type_is_float(T) else T{i = 0}
	one :=
		T(1) when intrinsics.type_is_float(T) else T {
			i = 1 << intrinsics.type_polymorphic_record_parameter_value(T, 1),
		}
	ten := NumConst(10, T)
	lo := zero
	hi := one

	for i in 0 ..< 64 {
		mid := NumMid(lo, hi)
		p := EvalBezier(kind, pts, mid)
		diff := Vec2Sub(p, point)
		if NumEqE(NumAbs(diff[0]), NumAbs(diff[1])) {
			return mid, true
		}

		// Use tangent direction to choose the next half interval.
		tangent := EvalBezierTangent(kind, pts, mid)
		if NumGt(Vec2Dot(tangent, diff), zero) {
			hi = mid
		} else {
			lo = mid
		}
	}

	mid := NumMid(lo, hi)
	p := EvalBezier(kind, pts, mid)
	diff := Vec2Sub(p, point)
	if NumEqE(NumAbs(diff[0]), NumAbs(diff[1])) {
		return mid, true
	}

	return zero, false
}

// Evaluates a point on a quadratic or cubic Bezier curve.
EvalBezier :: proc "contextless" (
	kind: BezierKind,
	pts: [4][2]$T,
	t: T,
) -> [2]T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	one :=
		T(1) when intrinsics.type_is_float(T) else T {
			i = 1 << intrinsics.type_polymorphic_record_parameter_value(T, 1),
		}
	u := NumSub(one, t)
	two := NumConst(2, T)
	three := NumConst(3, T)
	#partial switch kind {
	case .Quad:
		p0, p1, p2 := pts[0], pts[1], pts[2]
		uu := NumMul(u, u)
		tt := NumMul(t, t)
		ut2 := NumMul(NumMul(two, u), t)
		return {
			NumAdd(NumAdd(NumMul(uu, p0[0]), NumMul(ut2, p1[0])), NumMul(tt, p2[0])),
			NumAdd(NumAdd(NumMul(uu, p0[1]), NumMul(ut2, p1[1])), NumMul(tt, p2[1])),
		}
	case .Cubic:
		p0, p1, p2, p3 := pts[0], pts[1], pts[2], pts[3]
		uu := NumMul(u, u)
		tt := NumMul(t, t)
		uuu := NumMul(uu, u)
		ttt := NumMul(tt, t)
		uut3 := NumMul(NumMul(three, uu), t)
		utt3 := NumMul(NumMul(three, u), tt)
		return {
			NumAdd(
				NumAdd(NumAdd(NumMul(uuu, p0[0]), NumMul(uut3, p1[0])), NumMul(utt3, p2[0])),
				NumMul(ttt, p3[0]),
			),
			NumAdd(
				NumAdd(NumAdd(NumMul(uuu, p0[1]), NumMul(uut3, p1[1])), NumMul(utt3, p2[1])),
				NumMul(ttt, p3[1]),
			),
		}
	}
	return {}
}

// Evaluates a tangent on a quadratic or cubic Bezier curve.
EvalBezierTangent :: proc "contextless" (
	kind: BezierKind,
	pts: [4][2]$T,
	t: T,
) -> [2]T where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	one :=
		T(1) when intrinsics.type_is_float(T) else T {
			i = 1 << intrinsics.type_polymorphic_record_parameter_value(T, 1),
		}
	u := NumSub(one, t)
	two := NumConst(2, T)
	three := NumConst(3, T)
	six := NumConst(6, T)
	switch kind {
	case .Line:
		p0, p1 := pts[0], pts[1]
		return {NumSub(p1[0], p0[0]), NumSub(p1[1], p0[1])}
	case .Quad:
		p0, p1, p2 := pts[0], pts[1], pts[2]
		return {
			NumAdd(
				NumMul(NumMul(two, u), NumSub(p1[0], p0[0])),
				NumMul(NumMul(two, t), NumSub(p2[0], p1[0])),
			),
			NumAdd(
				NumMul(NumMul(two, u), NumSub(p1[1], p0[1])),
				NumMul(NumMul(two, t), NumSub(p2[1], p1[1])),
			),
		}
	case .Cubic:
		p0, p1, p2, p3 := pts[0], pts[1], pts[2], pts[3]
		uu := NumMul(u, u)
		tt := NumMul(t, t)
		ut6 := NumMul(NumMul(six, u), t)
		return {
			NumAdd(
				NumAdd(
					NumMul(NumMul(three, uu), NumSub(p1[0], p0[0])),
					NumMul(ut6, NumSub(p2[0], p1[0])),
				),
				NumMul(NumMul(three, tt), NumSub(p3[0], p2[0])),
			),
			NumAdd(
				NumAdd(
					NumMul(NumMul(three, uu), NumSub(p1[1], p0[1])),
					NumMul(ut6, NumSub(p2[1], p1[1])),
				),
				NumMul(NumMul(three, tt), NumSub(p3[1], p2[1])),
			),
		}
	}
	return {}
}

//return -1 if failed
GetBezierTForXMonotone :: proc(
	kind: BezierKind,
	pts: [4]$T,
) -> (
	t0: T,
	t1: T,
) where intrinsics.type_is_specialization_of(T, fixed.Fixed) ||
	intrinsics.type_is_float(T) {
	if kind == .Quad {
		// dx/dt = A + B·t = 0
		x0, x1, x2 := pts[0], pts[1], pts[2]
		A: T = NumAdd(NumMul(NumConst(-2, T), x0), NumMul(NumConst(2, T), x1))
		B: T = NumMul(NumConst(2, T), NumAdd(NumSub(x0, NumMul(NumConst(2, T), x1)), x2))

		if NumEq(B, NumConst(0, T)) do return NumConst(-1, T), NumConst(-1, T)
		return NumDiv(NumSign(A), B), NumConst(-1, T)
	} else if kind == .Cubic {
		// dx/dt = coef_A·t² + coef_B·t + coef_C = 0 (x component)
		two := NumConst(2, T)
		three := NumConst(3, T)
		six := NumConst(6, T)
		four := NumConst(4, T)
		x0, x1, x2, x3 := pts[0], pts[1], pts[2], pts[3]
		coefA := NumMul(
			three,
			NumAdd(NumSub(NumMul(three, x1), x0), NumSub(x3, NumMul(three, x2))),
		)
		coefB := NumMul(six, NumAdd(NumSub(x0, NumMul(two, x1)), x2))
		coefC := NumMul(three, NumSub(x1, x0))
		zero := NumConst(0, T)
		twoA := NumMul(two, coefA)

		if NumEq(coefA, zero) {
			if NumEq(coefB, zero) do return NumConst(-1, T), NumConst(-1, T)
			return NumDiv(NumSign(coefC), coefB), NumConst(-1, T)
		}

		D := NumSub(NumMul(coefB, coefB), NumMul(NumMul(four, coefA), coefC))
		if NumLt(D, zero) {
			return NumConst(-1, T), NumConst(-1, T)
		} else if NumEq(D, zero) {
			return NumDiv(NumSign(coefB), twoA), NumConst(-1, T)
		}

		sqrtD := NumSqrt(D)
		r0 := NumDiv(NumSub(NumSign(coefB), sqrtD), twoA)
		r1 := NumDiv(NumAdd(NumSign(coefB), sqrtD), twoA)
		return NumMin(r0, r1), NumMax(r0, r1)
	}
	return NumConst(-1, T), NumConst(-1, T)
}

@(test)
test2quadCurves :: proc(t: ^testing.T) {
	pt0: [4]linalg.Vector2f32 = {{0.0, 0.0}, {1.0, 0.0}, {1.0, -1.0}, {}} //last leave empty

	pt1: [4]linalg.Vector2f32 = {{1.0, 0.0}, {0.0, 0.0}, {0.0, -1.0}, {}} //last leave empty

	count, _, tAs, tBs := GetBezierIntersectPt(.Quad, pt0, .Quad, pt1)

	testing.expect_value(t, count == 1, true)
	testing.expect_value(t, tAs[0] == tBs[0], true)
	//fmt.println(ok, pt)
}

