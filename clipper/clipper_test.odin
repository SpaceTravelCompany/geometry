package clipper

import "../linalg_ex"
import "base:intrinsics"
import "core:fmt"
import "core:math"
import "core:math/rand"
import "core:testing"
import "shared:utils_private"

@(private)
_makeSquarePath :: proc(
	x0, y0, size: $T,
	invert := false,
	allocator := context.allocator,
) -> [][2]T where intrinsics.type_is_float(T) {
	result := utils_private.makeNonZeroedSlice([][2]T, 4, allocator)
	half := size * T(0.5)
	if !invert {
		result[0] = {x0 + half, y0 + half}
		result[1] = {x0 - half, y0 + half}
		result[2] = {x0 - half, y0 - half}
		result[3] = {x0 + half, y0 - half}
	} else {
		result[0] = {x0 - half, y0 - half}
		result[1] = {x0 + half, y0 - half}
		result[2] = {x0 + half, y0 + half}
		result[3] = {x0 - half, y0 + half}
	}
	return result
}

@(private)
_deletePaths :: proc(paths: [][][2]$T) {
	for p in paths do delete(p)
	delete(paths)
}

@(private)
_deleteFlags :: proc(paths: [][]bool) {
	for p in paths do delete(p)
	delete(paths)
}

@(private)
_hasCurveFlag :: proc(paths: [][]bool) -> bool {
	for p in paths {
		for v in p {
			if v do return true
		}
	}
	return false
}

@(test)
testBoolean2squareOps :: proc(t: ^testing.T) {
	square := _makeSquarePath(f64(0), 0, 100)
	defer delete(square)
	square2 := _makeSquarePath(f64(50), -50, 100)
	defer delete(square2)

	subjects := [][][2]f64{square}
	clips := [][][2]f64{square2}

	res, resOpen, err := BooleanOp(.Union, [][][2]f64{square, square2}, nil, nil)
	defer _deletePaths(res)
	defer _deletePaths(resOpen)
	testing.expect_value(t, err, nil)
	testing.expect(t, len(res) > 0)

	resI, resOpenI, errI := BooleanOp(.Intersection, subjects, clips, nil)
	defer _deletePaths(resI)
	defer _deletePaths(resOpenI)
	testing.expect_value(t, errI, nil)
	testing.expect_value(t, len(resI), 1)

	resD, resOpenD, errD := BooleanOp(.Difference, subjects, clips, nil)
	defer _deletePaths(resD)
	defer _deletePaths(resOpenD)
	testing.expect_value(t, errD, nil)
	testing.expect(t, len(resD) > 0)

	resX, resOpenX, errX := BooleanOp(.Xor, subjects, clips, nil)
	defer _deletePaths(resX)
	defer _deletePaths(resOpenX)
	testing.expect_value(t, errX, nil)
	testing.expect(t, len(resX) > 0)
}

@(test)
testBooleanGenericF32 :: proc(t: ^testing.T) {
	square := _makeSquarePath(f32(0), 0, 100)
	defer delete(square)
	square2 := _makeSquarePath(f32(50), -50, 100)
	defer delete(square2)

	res, resOpen, err := BooleanOp(.Intersection, [][][2]f32{square}, [][][2]f32{square2}, nil)
	defer _deletePaths(res)
	defer _deletePaths(resOpen)

	testing.expect_value(t, err, nil)
	testing.expect_value(t, len(res), 1)
}

@(test)
testFillRulesCompileAndRun :: proc(t: ^testing.T) {
	outer := _makeSquarePath(f64(0), 0, 100)
	defer delete(outer)
	inner := _makeSquarePath(f64(0), 0, 40, true)
	defer delete(inner)

	rules := [4]FillRule{.EvenOdd, .NonZero, .Positive, .Negative}
	for rule in rules {
		res, resOpen, err := BooleanOp(.Union, [][][2]f64{outer, inner}, nil, nil, rule)
		defer _deletePaths(res)
		defer _deletePaths(resOpen)
		testing.expect_value(t, err, nil)
	}
}

@(test)
testOpenSubjectClipping :: proc(t: ^testing.T) {
	clipRect := _makeSquarePath(f64(0), 0, 100)
	defer delete(clipRect)
	line := utils_private.makeNonZeroedSlice([][2]f64, 2)
	defer delete(line)
	line[0] = {-80, 0}
	line[1] = {80, 0}
	emptySubjects: [][][2]f64 = nil

	res, resOpen, err := BooleanOp(
		.Intersection,
		emptySubjects,
		[][][2]f64{clipRect},
		[][][2]f64{line},
	)
	defer _deletePaths(res)
	defer _deletePaths(resOpen)

	testing.expect_value(t, err, nil)
	testing.expect_value(t, len(res), 0)
	testing.expect_value(t, len(resOpen), 1)
}

@(test)
testCurveBooleanPreservesFlags :: proc(t: ^testing.T) {
	path1, isCurve1 := linalg_ex.CircleCubicInit([2]f64{0, 0}, 50)
	path2, isCurve2 := linalg_ex.CircleCubicInit([2]f64{30, 0}, 50)

	subjects := [][][2]f64{path1[:]}
	clips := [][][2]f64{path2[:]}
	subjectCurves := [][]bool{isCurve1[:]}
	clipCurves := [][]bool{isCurve2[:]}

	res, resOpen, resCurves, resOpenCurves, err := BooleanOpCurve(
		.Intersection,
		subjects,
		clips,
		nil,
		subjectCurves,
		clipCurves,
		nil,
	)
	defer _deletePaths(res)
	defer _deletePaths(resOpen)
	defer _deleteFlags(resCurves)
	defer _deleteFlags(resOpenCurves)

	testing.expect_value(t, err, nil)
	testing.expect(t, len(res) > 0)
	testing.expect(t, _hasCurveFlag(resCurves))
}

@(test)
testRectClipPolygonAndCurve :: proc(t: ^testing.T) {
	rect := linalg_ex.RectInit(f64(-25), 25, 25, -25)
	poly := _makeSquarePath(f64(0), 0, 100)
	defer delete(poly)

	res, resCurves, err := RectClip(rect, [][][2]f64{poly})
	defer _deletePaths(res)
	defer _deleteFlags(resCurves)
	testing.expect_value(t, err, nil)
	testing.expect_value(t, len(res), 1)

	circleRect := linalg_ex.RectInit(f64(-60), 10, 60, -60)
	circle, circleCurves := linalg_ex.CircleCubicInit([2]f64{0, 0}, 50)
	res2, res2Curves, err2 := RectClip(
		circleRect,
		[][][2]f64{circle[:]},
		[][]bool{circleCurves[:]},
	)
	defer _deletePaths(res2)
	defer _deleteFlags(res2Curves)
	testing.expect_value(t, err2, nil)
	testing.expect(t, len(res2) > 0)
	testing.expect(t, _hasCurveFlag(res2Curves))
}

@(private)
_testSignedArea :: proc(path: [][2]f64) -> f64 {
	if len(path) < 3 do return 0

	area := f64(0)
	for i in 0 ..< len(path) {
		j := (i + 1) % len(path)
		area += path[i].x * path[j].y - path[j].x * path[i].y
	}
	return area * 0.5
}

@(private)
_testReversePath :: proc(path: [][2]f64) {
	for i := 0; i < len(path) / 2; i += 1 {
		j := len(path) - 1 - i
		path[i], path[j] = path[j], path[i]
	}
}

@(private)
_testReverseCurvePath :: proc(path: [][2]f64, curves: []bool) {
	if len(path) == 0 || len(path) != len(curves) || len(path) % 3 != 0 {
		_testReversePath(path)
		return
	}

	tmpPath := make([][2]f64, len(path), context.temp_allocator)
	tmpCurves := make([]bool, len(curves), context.temp_allocator)
	anchorCount := len(path) / 3

	out := 0
	for step in 0 ..< anchorCount {
		seg := (anchorCount - 1 - step) * 3
		nextAnchor := (seg + 3) % len(path)

		tmpPath[out] = path[nextAnchor]
		tmpPath[out + 1] = path[seg + 2]
		tmpPath[out + 2] = path[seg + 1]
		tmpCurves[out] = false
		tmpCurves[out + 1] = true
		tmpCurves[out + 2] = true
		out += 3
	}

	for p, i in tmpPath do path[i] = p
	for c, i in tmpCurves do curves[i] = c
}

@(private)
_testPathContainsPath :: proc(parent, child: [][2]f64) -> bool {
	for p in child {
		if linalg_ex.PointInPolygon(p, parent) == .Outside {
			return false
		}
	}
	return true
}

@(private)
_makeRandomCurvePolygonAt :: proc(
	rng: rand.Generator,
	center: [2]f64,
	minRadius, maxRadius: f64,
	minAnchorCount, anchorCountSpan: int,
	positive := true,
	allocator := context.allocator,
) -> (
	path: [][2]f64,
	curves: []bool,
) {
	anchorCount := minAnchorCount + rand.int_max(anchorCountSpan, rng)
	anchors := make([][2]f64, anchorCount, context.temp_allocator)

	baseRadius := rand.float64_range(minRadius, maxRadius, rng)
	startAngle := rand.float64_range(0, math.TAU, rng)
	step := math.TAU / f64(anchorCount)

	for i in 0 ..< anchorCount {
		// 각도 순서를 유지해서 self-intersection 없는 contour를 만든다.
		jitter := rand.float64_range(-0.28, 0.28, rng) * step
		radius := baseRadius * rand.float64_range(0.58, 1.0, rng)
		angle := startAngle + f64(i) * step + jitter
		anchors[i] = {center.x + math.cos(angle) * radius, center.y + math.sin(angle) * radius}
	}

	if (_testSignedArea(anchors) > 0) != positive {
		_testReversePath(anchors)
	}

	path = make([][2]f64, anchorCount * 3, allocator)
	curves = make([]bool, anchorCount * 3, allocator)

	for i in 0 ..< anchorCount {
		next := (i + 1) % anchorCount
		a := anchors[i]
		b := anchors[next]
		d := b - a
		normal := [2]f64{-d.y, d.x}
		bulge := rand.float64_range(0.035, 0.13, rng)
		if rand.int_max(2, rng) == 0 do bulge = -bulge

		dst := i * 3
		path[dst] = a
		path[dst + 1] = a + d * 0.33 + normal * bulge
		path[dst + 2] = a + d * 0.67 + normal * bulge
		curves[dst] = false
		curves[dst + 1] = true
		curves[dst + 2] = true
	}

	if (_testSignedArea(path) > 0) != positive {
		_testReverseCurvePath(path, curves)
	}
	return
}

@(test)
testRectClipRandomStress :: proc(t: ^testing.T) {
	state := rand.create(u64(42))
	rng := rand.default_random_generator(&state)

	for iteration in 0 ..< 1000 {
		outerCenter := [2]f64 {
			rand.float64_range(-350, 350, rng),
			rand.float64_range(-350, 350, rng),
		}
		poly, polyCurves := _makeRandomCurvePolygonAt(rng, outerCenter, 180, 520, 5, 12, true)
		testing.expect(t, _testSignedArea(poly) > 0, "outer path must keep positive orientation")

		pathCount := 2 + rand.int_max(2, rng)
		paths := make([][][2]f64, pathCount, context.temp_allocator)
		pathCurves := make([][]bool, pathCount, context.temp_allocator)
		paths[0] = poly
		pathCurves[0] = polyCurves

		holeBase := math.sqrt(math.abs(_testSignedArea(poly)) / math.PI) * 0.16
		holeAngle := rand.float64_range(0, math.TAU, rng)
		for holeIdx in 1 ..< pathCount {
			holeOffset := holeBase * 0.75
			if pathCount == 2 do holeOffset = rand.float64_range(0, holeBase * 0.35, rng)
			angle := holeAngle + math.TAU * f64(holeIdx - 1) / f64(pathCount - 1)
			holeCenter := [2]f64 {
				outerCenter.x + math.cos(angle) * holeOffset,
				outerCenter.y + math.sin(angle) * holeOffset,
			}
			holePath, holeCurves := _makeRandomCurvePolygonAt(
				rng,
				holeCenter,
				holeBase * 0.22,
				holeBase * 0.34,
				4,
				6,
				false,
			)
			testing.expect(
				t,
				_testSignedArea(holePath) < 0,
				"hole path must keep negative orientation",
			)
			testing.expect(t, _testPathContainsPath(poly, holePath), "hole path must stay inside outer path")
			paths[holeIdx] = holePath
			pathCurves[holeIdx] = holeCurves
		}

		// 랜덤 clipRect 생성
		left: f64 = -100
		right: f64 = left + 200
		bottom: f64 = -100
		top: f64 = bottom + 200
		rect := linalg_ex.RectInit(left, right, top, bottom)

		// RectClip 호출
		res, resCurves, err := RectClip(rect, paths, pathCurves)

		for i in 0 ..< len(paths) {
			delete(paths[i])
			delete(pathCurves[i])
		}

		testing.expect_value(t, err == nil, true)

		// 결과 각 path 검사
		for path, pathIdx in res {
			// 경로 길이 확인 (최소 3점 필요)
			testing.expect(t, len(path) >= 3, "path too short")
			if len(resCurves) > pathIdx {
				testing.expect_value(t, len(resCurves[pathIdx]), len(path))
			}

			// 연속된 점이 같은 좌표가 아닌지 (zero-length edge 검사)
			for j in 0 ..< len(path) - 1 {
				if path[j] == path[j + 1] {
					testing.expectf(
						t,
						false,
						"zero-length segment at iter=%v path[%v]==path[%v] = (%v,%v)",
						iteration,
						j,
						j + 1,
						path[j].x,
						path[j].y,
					)
				}
			}

			// closed polygon: path[0] != path[last]
			if len(path) >= 3 && path[0] == path[len(path) - 1] {
				testing.expectf(
					t,
					false,
					"duplicate closure point at iter=%v: path[0] == path[last]",
					iteration,
				)
			}
		}

		_deletePaths(res)
		_deleteFlags(resCurves)
	}
}


// RectClip 경계에 정확히 맞닿는 vertex가 있는 폴리곤 테스트
@(test)
testRectClipVertexOnBoundary :: proc(t: ^testing.T) {
	// 폴리곤: 직사각형의 오른쪽 경계에 vertex가 정확히 있는 삼각형
	// rect: (-10, 10, 10, -10), poly: (10, 0), (0, 10), (0, -10) - 삼각형
	rect := linalg_ex.RectInit(f64(-10), 10, 10, -10)
	poly := [][2]f64{{10, 0}, {0, 10}, {0, -10}}
	res, _, err := RectClip(rect, [][][2]f64{poly})
	defer _deletePaths(res)
	testing.expect_value(t, err, nil)

	for path in res {
		testing.expect(t, len(path) >= 3, "path too short")
		for j in 0 ..< len(path) - 1 {
			if path[j] == path[j + 1] {
				testing.expectf(t, false, "zero-length segment at [%v]==[%v]", j, j + 1)
			}
		}
	}
}

// RectClip 경계와 collinear한 edge가 있는 폴리곤 테스트
@(test)
testRectClipCollinearEdge :: proc(t: ^testing.T) {
	// rect: (-10, 10, 10, -10), poly edge (0, 10) to (20, 10)은 rect의 top edge와 collinear
	rect := linalg_ex.RectInit(f64(-10), 10, 10, -10)
	poly := [][2]f64{{20, 10}, {20, -20}, {-20, -20}, {-20, 10}, {0, 10}}
	res, _, err := RectClip(rect, [][][2]f64{poly})
	defer _deletePaths(res)
	testing.expect_value(t, err, nil)

	for path in res {
		testing.expect(t, len(path) >= 3, "path too short")
		for j in 0 ..< len(path) - 1 {
			if path[j] == path[j + 1] {
				testing.expectf(
					t,
					false,
					"zero-length segment at [%v]==[%v]: (%v,%v)",
					j,
					j + 1,
					path[j].x,
					path[j].y,
				)
			}
		}
	}
}

// RectClip의 네 모서리를 모두 포함하는 큰 폴리곤 테스트
@(test)
testRectClipLargePolygon :: proc(t: ^testing.T) {
	rect := linalg_ex.RectInit(f64(-50), 50, 50, -50)
	square := _makeSquarePath(f64(0), 0, 200)
	defer delete(square)
	res, _, err := RectClip(rect, [][][2]f64{square})
	defer _deletePaths(res)
	testing.expect_value(t, err, nil)

	for path in res {
		testing.expect(t, len(path) >= 3, "path too short")
		for j in 0 ..< len(path) - 1 {
			if path[j] == path[j + 1] {
				testing.expectf(
					t,
					false,
					"zero-length segment at [%v]==[%v]: (%v,%v)",
					j,
					j + 1,
					path[j].x,
					path[j].y,
				)
			}
		}
	}
}

// 두 개의 폴리곤이 RectClip 경계에서 교차하는 케이스
@(test)
testRectClipMultiplePolygons :: proc(t: ^testing.T) {
	rect := linalg_ex.RectInit(f64(-100), 100, 100, -100)
	poly1 := _makeSquarePath(f64(0), 0, 150)
	defer delete(poly1)
	poly2 := _makeSquarePath(f64(80), 0, 100)
	defer delete(poly2)

	res, _, err := RectClip(rect, [][][2]f64{poly1, poly2})
	defer _deletePaths(res)
	testing.expect_value(t, err, nil)

	for path in res {
		testing.expect(t, len(path) >= 3, "path too short")
		for j in 0 ..< len(path) - 1 {
			if path[j] == path[j + 1] {
				testing.expectf(
					t,
					false,
					"zero-length segment at [%v]==[%v]: (%v,%v)",
					j,
					j + 1,
					path[j].x,
					path[j].y,
				)
			}
		}
	}
}

// 스트레스 테스트 - 더 많은 iteration + snap-point 기반 비교
@(test)
testRectClipRandomStressExt :: proc(t: ^testing.T) {
	state := rand.create(u64(12345))
	rng := rand.default_random_generator(&state)

	for iteration in 0 ..< 200 {
		numPts := 3 + rand.int_max(18, rng)
		poly := make([][2]f64, numPts)
		for j in 0 ..< numPts {
			poly[j] = {rand.float64_range(-1000, 1000, rng), rand.float64_range(-1000, 1000, rng)}
		}

		left := rand.float64_range(-800, 400, rng)
		right := left + rand.float64_range(20, 400, rng)
		bottom := rand.float64_range(-400, 800, rng)
		top := bottom + rand.float64_range(20, 400, rng)
		rect := linalg_ex.RectInit(left, right, top, bottom)

		res, _, err := RectClip(rect, [][][2]f64{poly})
		delete(poly)
		defer _deletePaths(res)

		if err != nil {
			continue
		}

		for path in res {
			testing.expect(t, len(path) >= 3, "path too short")
			for j in 0 ..< len(path) - 1 {
				if path[j] == path[j + 1] {
					testing.expectf(
						t,
						false,
						"zero-length segment at iter=%v j=%v: (%v,%v)",
						iteration,
						j,
						path[j].x,
						path[j].y,
					)
				}
			}
		}
	}
}

// rect의 한 변과 완전히 collinear한 폴리곤 에지 테스트
@(test)
testRectClipExactCollinear :: proc(t: ^testing.T) {
	// rect top edge: y=10, from x=-10 to x=10
	// poly edge: y=10, from x=-15 to x=15 (completely covers rect top and extends)
	rect := linalg_ex.RectInit(f64(-10), 10, 10, -10)
	poly := [][2]f64{{-20, 20}, {20, 20}, {20, 10}, {10, 10}, {-10, 10}, {-20, 10}}
	res, _, err := RectClip(rect, [][][2]f64{poly})
	defer _deletePaths(res)
	testing.expect_value(t, err, nil)

	for path in res {
		for j in 0 ..< len(path) - 1 {
			if path[j] == path[j + 1] {
				testing.expectf(
					t,
					false,
					"zero-length at [%v]==[%v]: (%v,%v)",
					j,
					j + 1,
					path[j].x,
					path[j].y,
				)
			}
		}
	}
}

// 좁은 직사각형이 여러 개 겹쳐서 RectClip 되는 케이스
@(test)
testRectClipNarrowOverlap :: proc(t: ^testing.T) {
	state := rand.create(u64(999))
	rng := rand.default_random_generator(&state)

	rect := linalg_ex.RectInit(f64(-100), 100, 100, -100)

	for iter in 0 ..< 50 {
		cx := rand.float64_range(-80, 80, rng)
		cy := rand.float64_range(-80, 80, rng)
		w := rand.float64_range(5, 50, rng)
		h := rand.float64_range(5, 50, rng)
		poly := [][2]f64 {
			{cx - w / 2, cy + h / 2},
			{cx + w / 2, cy + h / 2},
			{cx + w / 2, cy - h / 2},
			{cx - w / 2, cy - h / 2},
		}

		res, _, err := RectClip(rect, [][][2]f64{poly})
		defer _deletePaths(res)

		if err != nil {
			continue
		}

		for path in res {
			for j in 0 ..< len(path) - 1 {
				if path[j] == path[j + 1] {
					testing.expectf(
						t,
						false,
						"zero-length at iter=%v [%v]==[%v]: (%v,%v)",
						iter,
						j,
						j + 1,
						path[j].x,
						path[j].y,
					)
				}
			}
		}
	}
}

// snap point 비교로 zero-length 검사 (엄격한 일치가 아닌 snapping 기반)
_testSnapEqual :: proc(a, b: [2]f64) -> bool {
	SNAP_INV :: f64(1.0 / 1e-9)
	snap :: proc(p: [2]f64) -> u128 {
		x := u64(i64(math.round(p.x * SNAP_INV)))
		y := u64(i64(math.round(p.y * SNAP_INV)))
		return (u128(x) << 64) | u128(y)
	}
	return snap(a) == snap(b)
}

@(test)
testRectClipRandomStressSnap :: proc(t: ^testing.T) {
	state := rand.create(u64(77777))
	rng := rand.default_random_generator(&state)

	failCount := 0

	for iteration in 0 ..< 500 {
		numPts := 3 + rand.int_max(15, rng)
		poly := make([][2]f64, numPts)
		for j in 0 ..< numPts {
			poly[j] = {rand.float64_range(-500, 500, rng), rand.float64_range(-500, 500, rng)}
		}

		left := rand.float64_range(-400, 200, rng)
		right := left + rand.float64_range(20, 300, rng)
		bottom := rand.float64_range(-200, 400, rng)
		top := bottom + rand.float64_range(20, 300, rng)
		rect := linalg_ex.RectInit(left, right, top, bottom)

		res, _, err := RectClip(rect, [][][2]f64{poly})
		delete(poly)
		defer _deletePaths(res)

		if err != nil {
			continue
		}

		for path in res {
			if len(path) < 3 do continue
			for j in 0 ..< len(path) - 1 {
				if _testSnapEqual(path[j], path[j + 1]) {
					failCount += 1
					if failCount <= 5 {
						testing.expectf(
							t,
							false,
							"snap-zero-length at iter=%v [%v]==[%v]: (%v,%v) vs (%v,%v)",
							iteration,
							j,
							j + 1,
							path[j].x,
							path[j].y,
							path[j + 1].x,
							path[j + 1].y,
						)
					}
				}
			}
		}
	}

	if failCount > 0 {
		testing.expectf(t, false, "Total snap-zero-length failures: %v", failCount)
	}
}
