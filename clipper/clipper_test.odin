package clipper

import "../linalg_ex"
import "base:intrinsics"
import "core:testing"
import "shared:utils_private"

@(private)
_make_square_path :: proc(
	x0, y0, size: $T,
	invert := false,
	allocator := context.allocator,
) -> [][2]T where intrinsics.type_is_float(T) {
	result := utils_private.make_non_zeroed_slice([][2]T, 4, allocator)
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
_delete_paths :: proc(paths: [][][2]$T) {
	for p in paths do delete(p)
	delete(paths)
}

@(private)
_delete_flags :: proc(paths: [][]bool) {
	for p in paths do delete(p)
	delete(paths)
}

@(private)
_has_curve_flag :: proc(paths: [][]bool) -> bool {
	for p in paths {
		for v in p {
			if v do return true
		}
	}
	return false
}

@(test)
test_boolean_2square_ops :: proc(t: ^testing.T) {
	square := _make_square_path(f64(0), 0, 100)
	defer delete(square)
	square2 := _make_square_path(f64(50), -50, 100)
	defer delete(square2)

	subjects := [][][2]f64{square}
	clips := [][][2]f64{square2}

	res, res_open, err := BooleanOp(.Union, [][][2]f64{square, square2}, nil, nil)
	defer _delete_paths(res)
	defer _delete_paths(res_open)
	testing.expect_value(t, err, nil)
	testing.expect(t, len(res) > 0)

	res_i, res_open_i, err_i := BooleanOp(.Intersection, subjects, clips, nil)
	defer _delete_paths(res_i)
	defer _delete_paths(res_open_i)
	testing.expect_value(t, err_i, nil)
	testing.expect_value(t, len(res_i), 1)

	res_d, res_open_d, err_d := BooleanOp(.Difference, subjects, clips, nil)
	defer _delete_paths(res_d)
	defer _delete_paths(res_open_d)
	testing.expect_value(t, err_d, nil)
	testing.expect(t, len(res_d) > 0)

	res_x, res_open_x, err_x := BooleanOp(.Xor, subjects, clips, nil)
	defer _delete_paths(res_x)
	defer _delete_paths(res_open_x)
	testing.expect_value(t, err_x, nil)
	testing.expect(t, len(res_x) > 0)
}

@(test)
test_boolean_generic_f32 :: proc(t: ^testing.T) {
	square := _make_square_path(f32(0), 0, 100)
	defer delete(square)
	square2 := _make_square_path(f32(50), -50, 100)
	defer delete(square2)

	res, res_open, err := BooleanOp(.Intersection, [][][2]f32{square}, [][][2]f32{square2}, nil)
	defer _delete_paths(res)
	defer _delete_paths(res_open)

	testing.expect_value(t, err, nil)
	testing.expect_value(t, len(res), 1)
}

@(test)
test_fill_rules_compile_and_run :: proc(t: ^testing.T) {
	outer := _make_square_path(f64(0), 0, 100)
	defer delete(outer)
	inner := _make_square_path(f64(0), 0, 40, true)
	defer delete(inner)

	rules := [4]FillRule{.EvenOdd, .NonZero, .Positive, .Negative}
	for rule in rules {
		res, res_open, err := BooleanOp(.Union, [][][2]f64{outer, inner}, nil, nil, rule)
		defer _delete_paths(res)
		defer _delete_paths(res_open)
		testing.expect_value(t, err, nil)
	}
}

@(test)
test_open_subject_clipping :: proc(t: ^testing.T) {
	clip_rect := _make_square_path(f64(0), 0, 100)
	defer delete(clip_rect)
	line := utils_private.make_non_zeroed_slice([][2]f64, 2)
	defer delete(line)
	line[0] = {-80, 0}
	line[1] = {80, 0}
	empty_subjects: [][][2]f64 = nil

	res, res_open, err := BooleanOp(.Intersection, empty_subjects, [][][2]f64{clip_rect}, [][][2]f64{line})
	defer _delete_paths(res)
	defer _delete_paths(res_open)

	testing.expect_value(t, err, nil)
	testing.expect_value(t, len(res), 0)
	testing.expect_value(t, len(res_open), 1)
}

@(test)
test_curve_boolean_preserves_flags :: proc(t: ^testing.T) {
	path1, is_curve1 := linalg_ex.CircleCubicInit([2]f64{0, 0}, 50)
	path2, is_curve2 := linalg_ex.CircleCubicInit([2]f64{30, 0}, 50)

	subjects := [][][2]f64{path1[:]}
	clips := [][][2]f64{path2[:]}
	subject_curves := [][]bool{is_curve1[:]}
	clip_curves := [][]bool{is_curve2[:]}

	res, res_open, res_curves, res_open_curves, err := BooleanOpCurve(
		.Intersection,
		subjects,
		clips,
		nil,
		subject_curves,
		clip_curves,
		nil,
	)
	defer _delete_paths(res)
	defer _delete_paths(res_open)
	defer _delete_flags(res_curves)
	defer _delete_flags(res_open_curves)

	testing.expect_value(t, err, nil)
	testing.expect(t, len(res) > 0)
	testing.expect(t, _has_curve_flag(res_curves))
}

@(test)
test_rect_clip_polygon_and_curve :: proc(t: ^testing.T) {
	rect := linalg_ex.Rect_Init(f64(-25), 25, 25, -25)
	poly := _make_square_path(f64(0), 0, 100)
	defer delete(poly)

	res, res_curves, err := RectClip(rect, [][][2]f64{poly})
	defer _delete_paths(res)
	defer _delete_flags(res_curves)
	testing.expect_value(t, err, nil)
	testing.expect_value(t, len(res), 1)

	circle_rect := linalg_ex.Rect_Init(f64(-60), 10, 60, -60)
	circle, circle_curves := linalg_ex.CircleCubicInit([2]f64{0, 0}, 50)
	res2, res2_curves, err2 := RectClip(circle_rect, [][][2]f64{circle[:]}, [][]bool{circle_curves[:]})
	defer _delete_paths(res2)
	defer _delete_flags(res2_curves)
	testing.expect_value(t, err2, nil)
	testing.expect(t, len(res2) > 0)
	testing.expect(t, _has_curve_flag(res2_curves))
}
