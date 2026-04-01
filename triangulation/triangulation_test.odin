package triangulation

import "core:testing"
import "engine:utils_private"


@(private)
_make_square_path :: proc(
	x0, y0, size: f32,
	invent := false,
	allocator := context.allocator,
) -> [][2]f32 {
	result := utils_private.make_non_zeroed_slice([][2]f32, 4, allocator)
	half := size * 0.5
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
test_triangulation_2square :: proc(t: ^testing.T) {
	x0: f32 = 2.0
	y0: f32 = 0.0
	size: f32 = 100.0
	x1: f32 = 25.0
	y1: f32 = -25.0
	size2: f32 = 50.0

	square := _make_square_path(x0, y0, size)
	defer delete(square)

	square2 := _make_square_path(x1, y1, size2)
	defer delete(square2)

	poly := [][][2]f32{square, square2}
	indices, err := TrianguatePolygons(poly)
	defer delete(indices)

	testing.expect_value(t, err, nil)
	testing.expect_value(t, len(indices), 12)
}

