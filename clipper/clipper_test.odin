package clipper

import "../linalg_ex"
import "base:intrinsics"
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

	res, resOpen, err := BooleanOp(.Intersection, emptySubjects, [][][2]f64{clipRect}, [][][2]f64{line})
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
	res2, res2Curves, err2 := RectClip(circleRect, [][][2]f64{circle[:]}, [][]bool{circleCurves[:]})
	defer _deletePaths(res2)
	defer _deleteFlags(res2Curves)
	testing.expect_value(t, err2, nil)
	testing.expect(t, len(res2) > 0)
	testing.expect(t, _hasCurveFlag(res2Curves))
}
