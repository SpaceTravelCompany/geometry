package geometry

import "core:hash"
import "core:math"
import "core:math/fixed"
import "core:testing"
import "shared:utils_private"


// make_square_path returns a CCW square (4 vertices). Arguments in FixedDef.
@(private)
make_square_path :: proc(
	x0, y0, size: FixedDef,
	invent := false,
	allocator := context.allocator,
) -> [][2]FixedDef {
	result := utils_private.make_non_zeroed_slice([][2]FixedDef, 4, allocator)
	half := fixed.div(size, FixedDef{i = 2 << FIXED_SHIFT})
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

@(test)
test_triangulation_square :: proc(t: ^testing.T) {
	// square from (0,0) with edge length 100 in fixed units
	scale := i64(1 << FIXED_SHIFT)
	x0 := FixedDef {
		i = 0,
	}
	y0 := FixedDef {
		i = 0,
	}
	size := FixedDef {
		i = 100 * scale,
	}
	square := make_square_path(x0, y0, size)
	defer delete(square)

	poly := [][][2]FixedDef{square}
	indices, err := TrianguatePolygons_Fixed(poly)
	defer delete(indices)

	testing.expect_value(t, err, nil)
	// convex quad -> exactly 2 triangles (6 indices)
	testing.expect_value(t, len(indices), 6)
}

@(test)
test_triangulation_2square :: proc(t: ^testing.T) {
	// square from (0,0) with edge length 100 in fixed units
	scale := i64(1 << FIXED_SHIFT)
	x0 := FixedDef {
		i = 0,
	}
	y0 := FixedDef {
		i = 0,
	}
	size := FixedDef {
		i = 100 * scale,
	}
	x1 := FixedDef {
		i = 25 * scale,
	}
	y1 := FixedDef {
		i = -25 * scale,
	}
	size2 := FixedDef {
		i = 50 * scale,
	}
	square := make_square_path(x0, y0, size)
	defer delete(square)
	square2 := make_square_path(x1, y1, size2)
	defer delete(square2)

	poly := [][][2]FixedDef{square, square2}
	indices, err := TrianguatePolygons_Fixed(poly)
	defer delete(indices)

	testing.expect_value(t, err, nil)
	testing.expect_value(t, len(indices), 12)
}

@(test)
test_cross_product_sign_convention :: proc(t: ^testing.T) {
	scale := i64(1 << FIXED_SHIFT)

	p0 := [2]FixedDef{{i = 0}, {i = 0}}
	p1 := [2]FixedDef{{i = 1 * scale}, {i = 0}}
	p2 := [2]FixedDef{{i = 1 * scale}, {i = 1 * scale}}

	// p0 -> p1 -> p2 is a left turn (CCW)
	testing.expect_value(t, CrossProductSign(p0, p1, p2), 1)
	testing.expect_value(
		t,
		GetPolygonOrientation([][2]FixedDef{p0, p1, p2}),
		PolyOrientation.CounterClockwise,
	)

	// p0 -> p2 -> p1 is a right turn (CW)
	testing.expect_value(t, CrossProductSign(p0, p2, p1), -1)
	testing.expect_value(
		t,
		GetPolygonOrientation([][2]FixedDef{p0, p2, p1}),
		PolyOrientation.Clockwise,
	)
}

