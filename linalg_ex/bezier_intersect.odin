package linalg_ex

import "base:intrinsics"
import "core:math"
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
_addUnitRoot :: proc "contextless" (
	roots: ^[4]$T,
	count: ^int,
	root: T,
) where intrinsics.type_is_float(T) {
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
_solveLinearUnit :: proc "contextless" (
	roots: ^[4]$T,
	count: ^int,
	b, c: T,
) where intrinsics.type_is_float(T) {
	eps := _rootEps(T)
	if math.abs(b) <= eps do return
	_addUnitRoot(roots, count, -c / b)
}

@(private = "file")
_solveQuadraticUnit :: proc "contextless" (
	roots: ^[4]$T,
	count: ^int,
	a, b, c: T,
) where intrinsics.type_is_float(T) {
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
_solveCubicUnit :: proc "contextless" (
	roots: ^[4]$T,
	count: ^int,
	a, b, c, d: T,
) where intrinsics.type_is_float(T) {
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
_lineTForPoint :: proc "contextless" (
	line: [4][2]$T,
	p: [2]T,
) -> T where intrinsics.type_is_float(T) {
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
	p := EvalBezier(curveKind, curve, math.clamp(tCurve, T(0), T(1)))
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
) where intrinsics.type_is_float(T) {
	half := T(0.5)
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
) -> T where intrinsics.type_is_float(T) {
	return (px - ax) * dy - (py - ay) * dx
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
) where intrinsics.type_is_float(T) {
	one := T(1)
	zero := T(0)
	tLo = one
	tHi = zero

	for i in 0 ..= n {
		if d[i] >= dMin && d[i] <= dMax {
			ti := T(i) / T(n)
			if ti < tLo do tLo = ti
			if ti > tHi do tHi = ti
		}
	}

	for i in 0 ..= n {
		for j in i + 1 ..= n {
			ti := T(i) / T(n)
			tj := T(j) / T(n)
			dd := d[j] - d[i]
			if dd == zero do continue
			dt := tj - ti

			if (d[i] < dMin) != (d[j] < dMin) {
				tc := ti + (dMin - d[i]) / dd * dt
				if tc >= zero && tc <= one {
					if tc < tLo do tLo = tc
					if tc > tHi do tHi = tc
				}
			}
			if (d[i] > dMax) != (d[j] > dMax) {
				tc := ti + (dMax - d[i]) / dd * dt
				if tc >= zero && tc <= one {
					if tc < tLo do tLo = tc
					if tc > tHi do tHi = tc
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
	kindA:   BezierKind,
	kindB:   BezierKind,
	aLo:     T,
	aHi:     T,
	bLo:     T,
	bHi:     T,
	swapped: bool, // true when a/b correspond to original b/a
}

// Extract sub-curve for parameter range [t_lo, t_hi].
@(private = "file")
_extractSub :: proc "contextless" (
	kind: BezierKind,
	pts: [4][2]$T,
	tLo, tHi: T,
) -> [4][2]T where intrinsics.type_is_float(T) {
	one := T(1)
	zero := T(0)
	result := pts

	if tLo > zero {
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

	denom := one - tLo
	if denom <= zero do return result
	tAdj := (tHi - tLo) / denom
	if tAdj >= one do return result

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

// ─── Recursive AABB Subdivision 교차 검사 (빠른 버전) ───────────────
// Line+Line → 즉시 라인 교차
// 그 외 → AABB overlap → recursive de Casteljau → leaf line-line

DEPTH_LIMIT :: 20

@(private = "file")
_linesFromBezier :: proc "contextless" (kind: BezierKind, pts: [4][2]$T) -> (a, b: [2]T) where intrinsics.type_is_float(T) {
	o := bezierOrder(kind)
	return pts[0], pts[o - 1]
}

@(private = "file")
_controlPointAABB :: proc "contextless" (kind: BezierKind, pts: [4][2]$T) -> (minX, minY, maxX, maxY: T) where intrinsics.type_is_float(T) {
	n := bezierOrder(kind)
	minX = pts[0].x; maxX = pts[0].x
	minY = pts[0].y; maxY = pts[0].y
	for i in 1 ..< n {
		if pts[i].x < minX do minX = pts[i].x
		if pts[i].x > maxX do maxX = pts[i].x
		if pts[i].y < minY do minY = pts[i].y
		if pts[i].y > maxY do maxY = pts[i].y
	}
	return
}

// Segment-segment intersection returning parameters on both segments.
@(private = "file")
_segIntersectParams :: proc "contextless" (
	a0, a1, b0, b1: [2]$T,
) -> (
	tA, tB: T,
	p: [2]T,
	hit: bool,
) where intrinsics.type_is_float(T) {
	den := (b1.y - b0.y) * (a1.x - a0.x) - (b1.x - b0.x) * (a1.y - a0.y)
	if den == T(0) do return // parallel or collinear
	ua := ((b1.x - b0.x) * (a0.y - b0.y) - (b1.y - b0.y) * (a0.x - b0.x)) / den
	ub := ((a1.x - a0.x) * (a0.y - b0.y) - (a1.y - a0.y) * (a0.x - b0.x)) / den
	if ua < T(0) || ua > T(1) || ub < T(0) || ub > T(1) do return
	return ua, ub, {a0.x + ua * (a1.x - a0.x), a0.y + ua * (a1.y - a0.y)}, true
}

@(private = "file")
_lineLineIntersection :: proc "contextless" (a, b: [4][2]$T) -> (count: int, ips: [MaxBezierIntersections][2]T, tAs, tBs: [MaxBezierIntersections]T) where intrinsics.type_is_float(T) {
	tA, tB, p, hit := _segIntersectParams(a[0], a[1], b[0], b[1])
	if !hit do return
	ips[0] = p
	tAs[0] = tA
	tBs[0] = tB
	count = 1
	return
}

// Control points within tol of the chord, projecting inside the chord span.
@(private = "file")
_isFlat :: proc "contextless" (
	kind: BezierKind,
	pts: [4][2]$T,
	tol2: T,
) -> bool where intrinsics.type_is_float(T) {
	if kind == .Line do return true
	n := bezierOrder(kind)
	a := pts[0]
	b := pts[n - 1]
	dx := b.x - a.x
	dy := b.y - a.y
	len2 := dx * dx + dy * dy
	if len2 <= tol2 do return true
	for i in 1 ..< n - 1 {
		px := pts[i].x - a.x
		py := pts[i].y - a.y
		cr := px * dy - py * dx
		if cr * cr > tol2 * len2 do return false
		dot := px * dx + py * dy
		if dot < T(0) || dot > len2 do return false
	}
	return true
}

// Record a hit, merging duplicates reported by adjacent subdivision leaves.
@(private = "file")
_addCurveHit :: proc "contextless" (
	count: ^int,
	ips: ^[MaxBezierIntersections][2]$T,
	tAs, tBs: ^[MaxBezierIntersections]T,
	p: [2]T,
	tA, tB: T,
) where intrinsics.type_is_float(T) {
	dedupEps := T(1e-4)
	for i in 0 ..< count^ {
		if math.abs(tAs^[i] - tA) <= dedupEps && math.abs(tBs^[i] - tB) <= dedupEps do return
	}
	if count^ >= MaxBezierIntersections do return
	ips^[count^] = p
	tAs^[count^] = tA
	tBs^[count^] = tB
	count^ += 1
}

@(private = "file")
_intersectRecursive :: proc "contextless" (
	$T: typeid,
	kindA: BezierKind, aIn: [4][2]T, aLo, aHi: T,
	kindB: BezierKind, bIn: [4][2]T, bLo, bHi: T,
	flatTol2: T,
	count: ^int, ips: ^[MaxBezierIntersections][2]T, tAs, tBs: ^[MaxBezierIntersections]T,
	depth: int,
) where intrinsics.type_is_float(T) {
	if count^ >= MaxBezierIntersections do return

	// AABB overlap 체크
	aMinX, aMinY, aMaxX, aMaxY := _controlPointAABB(kindA, aIn)
	bMinX, bMinY, bMaxX, bMaxY := _controlPointAABB(kindB, bIn)
	if aMaxX < bMinX || bMaxX < aMinX || aMaxY < bMinY || bMaxY < aMinY do return

	// 양쪽 모두 충분히 평탄하면 chord-chord 교차로 종료
	if depth >= DEPTH_LIMIT || (_isFlat(kindA, aIn, flatTol2) && _isFlat(kindB, bIn, flatTol2)) {
		la0, la1 := _linesFromBezier(kindA, aIn)
		lb0, lb1 := _linesFromBezier(kindB, bIn)
		tA, tB, p, hit := _segIntersectParams(la0, la1, lb0, lb1)
		if hit {
			_addCurveHit(count, ips, tAs, tBs, p, aLo + tA * (aHi - aLo), bLo + tB * (bHi - bLo))
		}
		return
	}

	// 더 큰 AABB 쪽을 subdivision
	aDiag := (aMaxX - aMinX) * (aMaxX - aMinX) + (aMaxY - aMinY) * (aMaxY - aMinY)
	bDiag := (bMaxX - bMinX) * (bMaxX - bMinX) + (bMaxY - bMinY) * (bMaxY - bMinY)
	if bDiag > aDiag {
		bLeft, bRight := _splitHalf(kindB, bIn)
		bMid := (bLo + bHi) / 2
		_intersectRecursive(T, kindA, aIn, aLo, aHi, kindB, bLeft,  bLo, bMid, flatTol2, count, ips, tAs, tBs, depth + 1)
		_intersectRecursive(T, kindA, aIn, aLo, aHi, kindB, bRight, bMid, bHi, flatTol2, count, ips, tAs, tBs, depth + 1)
	} else {
		aLeft, aRight := _splitHalf(kindA, aIn)
		aMid := (aLo + aHi) / 2
		_intersectRecursive(T, kindA, aLeft,  aLo, aMid, kindB, bIn, bLo, bHi, flatTol2, count, ips, tAs, tBs, depth + 1)
		_intersectRecursive(T, kindA, aRight, aMid, aHi, kindB, bIn, bLo, bHi, flatTol2, count, ips, tAs, tBs, depth + 1)
	}
}

// 빠른 교차 검사.
// Line이 포함되면 해석적으로 처리(line-line / line-curve 다항식 근),
// curve-curve는 AABB overlap → de Casteljau 분할 → flatness 기반 chord-chord.
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
) where intrinsics.type_is_float(T) {
	if kindA == .Line {
		if kindB == .Line do return _lineLineIntersection(aIn, bIn)
		return _lineCurveIntersections(aIn, kindB, bIn)
	}
	if kindB == .Line {
		count, ips, tLine, tCurve := _lineCurveIntersections(bIn, kindA, aIn)
		return count, ips, tCurve, tLine
	}

	// flatness 허용 오차: 입력 스케일 대비 상대값
	aMinX, aMinY, aMaxX, aMaxY := _controlPointAABB(kindA, aIn)
	bMinX, bMinY, bMaxX, bMaxY := _controlPointAABB(kindB, bIn)
	scale := max(aMaxX - aMinX, aMaxY - aMinY, bMaxX - bMinX, bMaxY - bMinY)
	flatTol := scale * T(1e-6)
	_intersectRecursive(T, kindA, aIn, 0, 1, kindB, bIn, 0, 1, flatTol * flatTol, &count, &ips, &tAs, &tBs, 0)
	return
}

// ─── Sederberg-Nishita Bézier Clipping (레퍼런스) ──────────────────
GetBezierIntersectPtSlow :: proc(
	kindA: BezierKind,
	aIn: [4][2]$T,
	kindB: BezierKind,
	bIn: [4][2]T,
) -> (
	count: int,
	ips: [MaxBezierIntersections][2]T,
	tAs: [MaxBezierIntersections]T,
	tBs: [MaxBezierIntersections]T,
) where intrinsics.type_is_float(T) {
	if kindA == .Line {
		return _lineCurveIntersections(aIn, kindB, bIn)
	}
	if kindB == .Line {
		count, ips, tLine, tCurve := _lineCurveIntersections(bIn, kindA, aIn)
		return count, ips, tCurve, tLine
	}

	one := T(1)
	zero := T(0)
	eps := epsilon(T)
	epsHit := eps * 10
	fourFifth := T(0.8)

	Work :: _BezClipWork(T)
	stack := make([dynamic]Work, context.temp_allocator)
	append(&stack, Work {
		a       = aIn,
		b       = bIn,
		kindA   = kindA,
		kindB   = kindB,
		aLo     = zero,
		aHi     = one,
		bLo     = zero,
		bHi     = one,
		swapped = false,
	})

	for len(stack) > 0 {
		w := pop(&stack)

		aRange := w.aHi - w.aLo
		bRange := w.bHi - w.bLo

		if aRange <= eps && bRange <= eps {
			na := bezierOrder(w.kindA) - 1
			ip := [2]T{(w.a[0].x + w.a[na].x) * 0.5, (w.a[0].y + w.a[na].y) * 0.5}
			tA: T
			tB: T
			if !w.swapped {
				tA = (w.aLo + w.aHi) * 0.5
				tB = (w.bLo + w.bHi) * 0.5
			} else {
				tA = (w.bLo + w.bHi) * 0.5
				tB = (w.aLo + w.aHi) * 0.5
			}

			isDup := false
			for i in 0 ..< count {
				if math.abs(tAs[i] - tA) <= epsHit &&
				   math.abs(tBs[i] - tB) <= epsHit {
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
		dx := w.a[na].x - ax
		dy := w.a[na].y - ay

		if dx == zero && dy == zero {
			continue
		}

		dMin := zero
		dMax := zero
		for i in 1 ..< na {
			d := (w.a[i].x - ax) * dy - (w.a[i].y - ay) * dx
			if d < dMin do dMin = d
			if d > dMax do dMax = d
		}

		nb := bezierOrder(w.kindB) - 1
		db: [4]T
		for i in 0 ..= nb {
			db[i] = (w.b[i].x - ax) * dy - (w.b[i].y - ay) * dx
		}

		clipLo, clipHi := _clipHull(db, nb, dMin, dMax)
		if clipLo > clipHi do continue

		if clipLo < zero do clipLo = zero
		if clipHi > one do clipHi = one

		bSpan := w.bHi - w.bLo
		newBLo := w.bLo + clipLo * bSpan
		newBHi := w.bLo + clipHi * bSpan
		bClipped := _extractSub(w.kindB, w.b, clipLo, clipHi)

		clipRange := clipHi - clipLo

		if clipRange > fourFifth {
			if aRange >= bRange {
				aLeft, aRight := _splitHalf(w.kindA, w.a)
				aMid := (w.aLo + w.aHi) * 0.5
				append(&stack, Work {
					a       = bClipped,
					b       = aLeft,
					kindA   = w.kindB,
					kindB   = w.kindA,
					aLo     = newBLo,
					aHi     = newBHi,
					bLo     = w.aLo,
					bHi     = aMid,
					swapped = !w.swapped,
				})
				append(&stack, Work {
					a       = bClipped,
					b       = aRight,
					kindA   = w.kindB,
					kindB   = w.kindA,
					aLo     = newBLo,
					aHi     = newBHi,
					bLo     = aMid,
					bHi     = w.aHi,
					swapped = !w.swapped,
				})
			} else {
				bLeft, bRight := _splitHalf(w.kindB, bClipped)
				bMid := (newBLo + newBHi) * 0.5
				append(&stack, Work {
					a       = bLeft,
					b       = w.a,
					kindA   = w.kindB,
					kindB   = w.kindA,
					aLo     = newBLo,
					aHi     = bMid,
					bLo     = w.aLo,
					bHi     = w.aHi,
					swapped = !w.swapped,
				})
				append(&stack, Work {
					a       = bRight,
					b       = w.a,
					kindA   = w.kindB,
					kindB   = w.kindA,
					aLo     = bMid,
					aHi     = newBHi,
					bLo     = w.aLo,
					bHi     = w.aHi,
					swapped = !w.swapped,
				})
			}
		} else {
			append(&stack, Work {
				a       = bClipped,
				b       = w.a,
				kindA   = w.kindB,
				kindB   = w.kindA,
				aLo     = newBLo,
				aHi     = newBHi,
				bLo     = w.aLo,
				bHi     = w.aHi,
				swapped = !w.swapped,
			})
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
) where intrinsics.type_is_float(T) {
	zero := T(0)
	one := T(1)
	lo := zero
	hi := one

	eps := epsilon(T) * T(16)
	for i in 0 ..< 64 {
		mid := (lo + hi) / T(2)
		p := EvalBezier(kind, pts, mid)
		diff := p - point
		if math.abs(diff[0]) <= eps && math.abs(diff[1]) <= eps {
			return mid, true
		}

		// Use tangent direction to choose the next half interval.
		tangent := EvalBezierTangent(kind, pts, mid)
		if tangent[0] * diff[0] + tangent[1] * diff[1] > zero {
			hi = mid
		} else {
			lo = mid
		}
	}

	mid := (lo + hi) / T(2)
	p := EvalBezier(kind, pts, mid)
	diff := p - point
	if math.abs(diff[0]) <= eps && math.abs(diff[1]) <= eps {
		return mid, true
	}

	return zero, false
}

// Evaluates a point on a line, quadratic or cubic Bezier curve.
EvalBezier :: proc "contextless" (
	kind: BezierKind,
	pts: [4][2]$T,
	t: T,
) -> [2]T where intrinsics.type_is_float(T) {
	u := T(1) - t
	two := T(2)
	three := T(3)
	switch kind {
	case .Line:
		p0, p1 := pts[0], pts[1]
		return {u * p0[0] + t * p1[0], u * p0[1] + t * p1[1]}
	case .Quad:
		p0, p1, p2 := pts[0], pts[1], pts[2]
		uu := u * u
		tt := t * t
		ut2 := two * u * t
		return {
			uu * p0[0] + ut2 * p1[0] + tt * p2[0],
			uu * p0[1] + ut2 * p1[1] + tt * p2[1],
		}
	case .Cubic:
		p0, p1, p2, p3 := pts[0], pts[1], pts[2], pts[3]
		uu := u * u
		tt := t * t
		uuu := uu * u
		ttt := tt * t
		uut3 := three * uu * t
		utt3 := three * u * tt
		return {
			uuu * p0[0] + uut3 * p1[0] + utt3 * p2[0] + ttt * p3[0],
			uuu * p0[1] + uut3 * p1[1] + utt3 * p2[1] + ttt * p3[1],
		}
	}
	return {}
}

// Evaluates a tangent on a quadratic or cubic Bezier curve.
EvalBezierTangent :: proc "contextless" (
	kind: BezierKind,
	pts: [4][2]$T,
	t: T,
) -> [2]T where intrinsics.type_is_float(T) {
	one := T(1)
	u := one - t
	two := T(2)
	three := T(3)
	six := T(6)
	switch kind {
	case .Line:
		p0, p1 := pts[0], pts[1]
		return {p1[0] - p0[0], p1[1] - p0[1]}
	case .Quad:
		p0, p1, p2 := pts[0], pts[1], pts[2]
		return {
			two * u * (p1[0] - p0[0]) + two * t * (p2[0] - p1[0]),
			two * u * (p1[1] - p0[1]) + two * t * (p2[1] - p1[1]),
		}
	case .Cubic:
		p0, p1, p2, p3 := pts[0], pts[1], pts[2], pts[3]
		uu := ((u) * (u))
		tt := ((t) * (t))
		ut6 := six * u * t
		return {
			three * uu * (p1[0] - p0[0]) + ut6 * (p2[0] - p1[0]) + three * tt * (p3[0] - p2[0]),
			three * uu * (p1[1] - p0[1]) + ut6 * (p2[1] - p1[1]) + three * tt * (p3[1] - p2[1]),
		}
	}
	return {}
}

//return -1 if failed
GetBezierTForXMonotone :: proc "contextless" (
	kind: BezierKind,
	pts: [4]$T,
) -> (
	t0: T,
	t1: T,
) where intrinsics.type_is_float(T) {
	if kind == .Quad {
		// dx/dt = A + B·t = 0
		x0, x1, x2 := pts[0], pts[1], pts[2]
		A := T(2) * (x1 - x0)
		B := T(2) * (x0 - T(2) * x1 + x2)

		if B == T(0) do return T(-1), T(-1)
		return -A / B, T(-1)
	} else if kind == .Cubic {
		// dx/dt = coef_A·t² + coef_B·t + coef_C = 0 (x component)
		two := T(2)
		three := T(3)
		six := T(6)
		four := T(4)
		x0, x1, x2, x3 := pts[0], pts[1], pts[2], pts[3]
		coefA := three * (three * x1 - x0 + x3 - three * x2)
		coefB := six * (x0 - two * x1 + x2)
		coefC := three * (x1 - x0)
		zero := T(0)
		twoA := two * coefA

		if coefA == zero {
			if coefB == zero do return T(-1), T(-1)
			return -coefC / coefB, T(-1)
		}

		D := coefB * coefB - four * coefA * coefC
		if D < zero {
			return T(-1), T(-1)
		} else if D == zero {
			return -coefB / twoA, T(-1)
		}

		sqrtD := math.sqrt(D)
		r0 := (-coefB - sqrtD) / twoA
		r1 := (-coefB + sqrtD) / twoA
		return min(r0, r1), max(r0, r1)
	}
	return T(-1), T(-1)
}

@(test)
test2quadCurves :: proc(t: ^testing.T) {
	pt0: [4]linalg.Vector2f32 = {{0.0, 0.0}, {1.0, 0.0}, {1.0, -1.0}, {}} //last leave empty
	pt1: [4]linalg.Vector2f32 = {{1.0, 0.0}, {0.0, 0.0}, {0.0, -1.0}, {}} //last leave empty

	count, ips, tAs, tBs := GetBezierIntersectPt(.Quad, pt0, .Quad, pt1)

	testing.expect_value(t, count, 1)
	// Mirrored curves cross at x = 0.5 with t = 1 - sqrt(2)/2 on both.
	expectedT := f32(1.0 - math.SQRT_TWO / 2.0)
	testing.expect(t, math.abs(ips[0].x - 0.5) < 1e-4)
	testing.expect(t, math.abs(tAs[0] - expectedT) < 1e-3)
	testing.expect(t, math.abs(tAs[0] - tBs[0]) < 1e-4)
}

@(test)
testLineLineIntersect :: proc(t: ^testing.T) {
	a: [4]linalg.Vector2f32 = {{0, 0}, {2, 2}, {}, {}}
	b: [4]linalg.Vector2f32 = {{0, 2}, {2, 0}, {}, {}}

	count, ips, tAs, tBs := GetBezierIntersectPt(.Line, a, .Line, b)

	testing.expect_value(t, count, 1)
	testing.expect(t, math.abs(ips[0].x - 1) < 1e-6)
	testing.expect(t, math.abs(ips[0].y - 1) < 1e-6)
	testing.expect(t, math.abs(tAs[0] - 0.5) < 1e-6)
	testing.expect(t, math.abs(tBs[0] - 0.5) < 1e-6)
}

@(test)
testLineQuadIntersect :: proc(t: ^testing.T) {
	// y(t) = 4t(1-t), peaks at 1; line y = 0.5 crosses twice.
	q: [4]linalg.Vector2f32 = {{0, 0}, {1, 2}, {2, 0}, {}}
	l: [4]linalg.Vector2f32 = {{-1, 0.5}, {3, 0.5}, {}, {}}

	count, ips, _, tBs := GetBezierIntersectPt(.Line, l, .Quad, q)

	testing.expect_value(t, count, 2)
	for i in 0 ..< count {
		testing.expect(t, math.abs(ips[i].y - 0.5) < 1e-5)
		// roots of 4t(1-t) = 0.5
		r0 := f32(0.5 - math.SQRT_TWO / 4.0)
		r1 := f32(0.5 + math.SQRT_TWO / 4.0)
		testing.expect(t, math.abs(tBs[i] - r0) < 1e-4 || math.abs(tBs[i] - r1) < 1e-4)
	}
}

@(test)
testBezierExtremaT :: proc(t: ^testing.T) {
	// Quad: x(t) extremum of {0, 2, 0} at t = 0.5
	qt0, qt1 := GetBezierTForXMonotone(.Quad, [4]f32{0, 2, 0, 0})
	testing.expect(t, math.abs(qt0 - 0.5) < 1e-6)
	testing.expect_value(t, qt1, f32(-1))

	// Cubic: x'(t) = 30t² - 30t + 6, roots 0.5 ± sqrt(180)/60
	ct0, ct1 := GetBezierTForXMonotone(.Cubic, [4]f32{0, 2, -1, 1})
	testing.expect(t, math.abs(ct0 - 0.276393) < 1e-4)
	testing.expect(t, math.abs(ct1 - 0.723607) < 1e-4)

	// Cubic degenerating to quadratic derivative (coefA == 0): x(t) = 3t(1-t)
	lt0, lt1 := GetBezierTForXMonotone(.Cubic, [4]f32{0, 1, 1, 0})
	testing.expect(t, math.abs(lt0 - 0.5) < 1e-6)
	testing.expect_value(t, lt1, f32(-1))
}

@(test)
testBezierAABBQuad :: proc(t: ^testing.T) {
	// Curve max y is 1 (control point at 2 lies outside the curve).
	q: [4]linalg.Vector2f32 = {{0, 0}, {1, 2}, {2, 0}, {}}
	bbox := BezierAABB(.Quad, q)
	testing.expect(t, math.abs(bbox.left - 0) < 1e-6)
	testing.expect(t, math.abs(bbox.right - 2) < 1e-6)
	testing.expect(t, math.abs(bbox.top - 1) < 1e-6)
	testing.expect(t, math.abs(bbox.bottom - 0) < 1e-6)
}

@(test)
testPointInCurvedPolygonSquare :: proc(t: ^testing.T) {
	edges := [4][4][2]f32 {
		{{0, 0}, {2, 0}, {}, {}},
		{{2, 0}, {2, 2}, {}, {}},
		{{2, 2}, {0, 2}, {}, {}},
		{{0, 2}, {0, 0}, {}, {}},
	}
	kinds := [4]BezierKind{.Line, .Line, .Line, .Line}

	testing.expect_value(t, PointInCurvedPolygon([2]f32{1, 1}, edges[:], kinds[:]), PointInPolygonResult.Inside)
	testing.expect_value(t, PointInCurvedPolygon([2]f32{3, 1}, edges[:], kinds[:]), PointInPolygonResult.Outside)
	testing.expect_value(t, PointInCurvedPolygon([2]f32{2, 1}, edges[:], kinds[:]), PointInPolygonResult.On)
}

/*
곡선의 exact axis-aligned bounding box 계산.

x(t), y(t)의 극값(extremum)을 해석적으로 구하고,
모든 극값 t와 t=0, t=1에서의 값을 평가하여 min/max 계산.
*/
BezierAABB :: proc "contextless" (kind: BezierKind, pts: [4][2]$T) -> Rect_(T) where intrinsics.type_is_float(T) {
	n := bezierOrder(kind)
	p0 := pts[0]
	p1 := pts[n - 1]

	minX := min(p0.x, p1.x); maxX := max(p0.x, p1.x)
	minY := min(p0.y, p1.y); maxY := max(p0.y, p1.y)

	if kind != .Line {
		xCoords: [4]T = {pts[0].x, pts[1].x, pts[2].x, pts[3].x}
		yCoords: [4]T = {pts[0].y, pts[1].y, pts[2].y, pts[3].y}
		tx0, tx1 := GetBezierTForXMonotone(kind, xCoords)
		ty0, ty1 := GetBezierTForXMonotone(kind, yCoords)
		for t in ([4]T{tx0, tx1, ty0, ty1}) {
			if t <= T(0) || t >= T(1) do continue // endpoints already covered
			pt := EvalBezier(kind, pts, t)
			minX = min(minX, pt.x); maxX = max(maxX, pt.x)
			minY = min(minY, pt.y); maxY = max(maxY, pt.y)
		}
	}

	return Rect_(T){left = minX, right = maxX, top = maxY, bottom = minY}
}

/*
곡선 에지로 구성된 폴리곤에 대한 점 포함 판정.
Ray casting (even-odd rule) 방식 — 오른쪽으로 수평선 ray를 발사하여
GetBezierIntersectPt로 각 곡선 에지와의 교차점을 계산.

edges: 곡선 제어점 배열
kinds: 각 에지의 타입 (BezierKind)
*/
PointInCurvedPolygon :: proc(
	p: [2]$T,
	edges: [][4][2]T,
	kinds: []BezierKind,
) -> PointInPolygonResult where intrinsics.type_is_float(T) {
	n := len(edges)
	if n < 3 do return .Outside

	eps := _rootEps(T)

	bboxes := make([]Rect_(T), n, context.temp_allocator)
	maxRight := p.x
	for i in 0 ..< n {
		bboxes[i] = BezierAABB(kinds[i], edges[i])
		if bboxes[i].right > maxRight do maxRight = bboxes[i].right
	}
	FAR := maxRight + T(100)

	ray: [4][2]T = {p, {FAR, p.y}, {}, {}}

	crossing := 0
	for i in 0 ..< n {
		bbox := bboxes[i]
		if p.y < bbox.bottom - eps || p.y > bbox.top + eps do continue
		if p.x > bbox.right do continue

		count, ips, _, _ := GetBezierIntersectPt(.Line, ray, kinds[i], edges[i])

		for j in 0 ..< count {
			pt := ips[j]
			if math.abs(pt.x - p.x) <= eps && math.abs(pt.y - p.y) <= eps {
				return .On
			}
			if pt.x >= p.x {
				crossing += 1
			}
		}
	}

	return (crossing % 2) == 1 ? .Inside : .Outside
}
