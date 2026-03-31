package clipper

import "../linalg_ex"
import "base:intrinsics"
import "core:fmt"
import "core:math/fixed"
import "core:testing"
import "engine:utils_private"


@(private)
_make_square_path :: proc(
	x0, y0, size: FixedDef,
	invent := false,
	allocator := context.allocator,
) -> [][2]FixedDef {
	result := utils_private.make_non_zeroed_slice([][2]FixedDef, 4, allocator)
	two: FixedDef = FixedDef {
		i = 2 << FixedDef.Fraction_Width,
	}
	half := fixed.div(size, two)
	if !invent {
		result[0] = {fixed.add(x0, half), fixed.add(y0, half)}
		result[1] = {fixed.sub(x0, half), fixed.add(y0, half)}
		result[2] = {fixed.sub(x0, half), fixed.sub(y0, half)}
		result[3] = {fixed.add(x0, half), fixed.sub(y0, half)}
	} else {
		result[0] = {fixed.sub(x0, half), fixed.sub(y0, half)}
		result[1] = {fixed.add(x0, half), fixed.sub(y0, half)}
		result[2] = {fixed.add(x0, half), fixed.add(y0, half)}
		result[3] = {fixed.sub(x0, half), fixed.add(y0, half)}
	}
	return result
}

@(private)
_make_square_path_float :: proc(
	x0, y0, size: $T,
	invent := false,
	allocator := context.allocator,
) -> [][2]T where intrinsics.type_is_float(T) {
	result := utils_private.make_non_zeroed_slice([][2]T, 4, allocator)
	two :: T(2)
	half := T(size) / two
	if !invent {
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

@(test)
test_union_2square :: proc(t: ^testing.T) {
	x0 := FixedDef {
		i = 0,
	}
	y0 := FixedDef {
		i = 0,
	}
	size := FixedDef {
		i = 100 << FixedDef.Fraction_Width,
	}

	x1 := FixedDef {
		i = 50 << FixedDef.Fraction_Width,
	}
	y1 := FixedDef {
		i = -50 << FixedDef.Fraction_Width,
	}
	size2 := FixedDef {
		i = 100 << FixedDef.Fraction_Width,
	}

	square := _make_square_path(x0, y0, size)
	defer delete(square)

	square2 := _make_square_path(x1, y1, size2)
	defer delete(square2)

	poly := [][][2]FixedDef{square, square2}
	res, res_open, err := BooleanOp_Fixed(.Union, poly, nil, nil)
	defer if res != nil {
		for r in res {
			delete(r)
		}
		delete(res)
	}
	defer if res_open != nil {
		for r in res_open {
			delete(r)
		}
		delete(res_open)
	}

	testing.expect_value(t, err, nil)
}

@(test)
test_union_2square_float :: proc(t: ^testing.T) {
	x0: f32 = 0.0
	y0: f32 = 0.0
	size: f32 = 100.0

	x1: f32 = 50.0
	y1: f32 = -50.0
	size2: f32 = 100.0

	square := _make_square_path_float(x0, y0, size)
	defer delete(square)

	square2 := _make_square_path_float(x1, y1, size2)
	defer delete(square2)

	poly := [][][2]f32{square, square2}
	res, res_open, err := BooleanOp(.Union, f32, poly, nil, nil)
	defer if res != nil {
		for r in res {
			delete(r)
		}
		delete(res)
	}
	defer if res_open != nil {
		for r in res_open {
			delete(r)
		}
		delete(res_open)
	}

	testing.expect_value(t, err, nil)
}

@(test)
test_intersection_2square :: proc(t: ^testing.T) {
	x0 := FixedDef {
		i = 0,
	}
	y0 := FixedDef {
		i = 0,
	}
	size := FixedDef {
		i = 100 << FixedDef.Fraction_Width,
	}

	x1 := FixedDef {
		i = 50 << FixedDef.Fraction_Width,
	}
	y1 := FixedDef {
		i = -50 << FixedDef.Fraction_Width,
	}
	size2 := FixedDef {
		i = 100 << FixedDef.Fraction_Width,
	}

	square := _make_square_path(x0, y0, size)
	defer delete(square)

	square2 := _make_square_path(x1, y1, size2)
	defer delete(square2)

	poly := [][][2]FixedDef{square}
	clip_poly := [][][2]FixedDef{square2}
	res, res_open, err := BooleanOp_Fixed(.Intersection, poly, clip_poly, nil)
	defer if res != nil {
		for r in res {
			delete(r)
		}
		delete(res)
	}
	defer if res_open != nil {
		for r in res_open {
			delete(r)
		}
		delete(res_open)
	}

	testing.expect_value(t, err, nil)
	testing.expect_value(t, len(res), 1)
	testing.expect_value(t, len(res[0]), 4)
}

@(test)
test_union_2circle_curve :: proc(t: ^testing.T) {
	r := FixedDef {
		i = 50 << FixedDef.Fraction_Width,
	}

	cx1 := FixedDef {
		i = 0,
	}
	cy1 := FixedDef {
		i = 0,
	}

	cx2 := FixedDef {
		i = 30 << FixedDef.Fraction_Width,
	}
	cy2 := FixedDef {
		i = 0,
	}

	path1, is_curve1 := linalg_ex.CircleCubicInit_Fixed([2]FixedDef{cx1, cy1}, r)
	path2, is_curve2 := linalg_ex.CircleCubicInit_Fixed([2]FixedDef{cx2, cy2}, r)

	// subjects := [][][2]FixedDef{path1[:], path2[:]}
	// subjects_is_curves := [][]bool{is_curve1[:], is_curve2[:]}

	// res, res_open, res_is_curves, res_open_is_curves, err := BooleanOpCurve_Fixed(
	// 	.Union,
	// 	subjects,
	// 	nil,
	// 	nil,
	// 	subjects_is_curves,
	// 	nil,
	// 	nil,
	// )
	// defer {
	// 	if res != nil {
	// 		for r in res do delete(r)
	// 		delete(res)
	// 	}
	// 	if res_open != nil {
	// 		for r in res_open do delete(r)
	// 		delete(res_open)
	// 	}
	// 	if res_is_curves != nil {
	// 		for r in res_is_curves do delete(r)
	// 		delete(res_is_curves)
	// 	}
	// 	if res_open_is_curves != nil {
	// 		for r in res_open_is_curves do delete(r)
	// 		delete(res_open_is_curves)
	// 	}
	// }
	// // for r in res {
	// // 	for p, i in r {
	// // 		fmt.printfln(
	// // 			"%d (%f, %f) %v",
	// // 			i,
	// // 			fixed.to_f64(p.x),
	// // 			fixed.to_f64(p.y),
	// // 			res_is_curves[0][i],
	// // 		)
	// // 	}
	// // }

	// testing.expect_value(t, err, nil)
	// testing.expect(t, len(res) == 1)
	// testing.expect(t, len(res_is_curves) == 1)
	// testing.expect(t, len(res[0]) == len(res_is_curves[0]))
}

@(test)
test_custom :: proc(t: ^testing.T) {
	poly2 := [5][2]FixedDef {
		{
			FixedDef{i = 340 << FixedDef.Fraction_Width},
			FixedDef{i = 60 << FixedDef.Fraction_Width},
		}, // 0: 꼭대기
		{
			FixedDef{i = 460 << FixedDef.Fraction_Width},
			FixedDef{i = 380 << FixedDef.Fraction_Width},
		}, // 1: 오른쪽 아래 (→ 자가교차)
		{
			FixedDef{i = 180 << FixedDef.Fraction_Width},
			FixedDef{i = 200 << FixedDef.Fraction_Width},
		}, // 2: 왼쪽
		{
			FixedDef{i = 500 << FixedDef.Fraction_Width},
			FixedDef{i = 200 << FixedDef.Fraction_Width},
		}, // 3: 오른쪽
		{
			FixedDef{i = 220 << FixedDef.Fraction_Width},
			FixedDef{i = 380 << FixedDef.Fraction_Width},
		}, // 4: 왼쪽 아래 (→ 자가교차)
	}

	res, res_open, err := BooleanOp_Fixed(.Union, [][][2]FixedDef{poly2[:]}, nil, nil, .NonZero)
	defer {
		for r in res {
			delete(r)
		}
		delete(res)
	}

	testing.expect_value(t, err, nil)
	testing.expect_value(t, len(res), 1)
	testing.expect_value(t, len(res[0]), 10)
}

