#+feature using-stmt
package geometry

import "base:intrinsics"
import "base:runtime"
import "core:math"
import "core:math/linalg"
import "core:mem"

import "core:math/fixed"

import "shared:utils_private"

__Trianguate_Error :: enum {
	FAILED,
}

Trianguate_Error :: union #shared_nil {
    __Trianguate_Error,
    runtime.Allocator_Error,
}

TrianguatePolygons_Fixed :: proc(poly:[][][2]FixedDef, polyCCW:[]PolyOrientation = nil, allocator := context.allocator) ->
(indices:[]u32, err:Trianguate_Error) {
	using fixed
	
	assert(len(poly) == len(polyCCW))
	
	indices_ := make([dynamic]u32, context.temp_allocator) or_return
	
	//polyCCW 정보 (폴리곤이 구멍인지 아닌지 여부) 가 없으면 직접 만들어야 한다.
	polyCCW := polyCCW
	if polyCCW == nil {
		polyCCW = utils_private.make_non_zeroed_slice([]PolyOrientation, len(poly), context.temp_allocator) or_return
		for &p, i in polyCCW {
			p = GetPolygonOrientation(poly[i])
		}
	}
	
	//TODO non_curves trianglation
	
	indices = utils_private.make_non_zeroed_slice([]u32, len(indices_), allocator) or_return
	mem.copy_non_overlapping(raw_data(indices), raw_data(indices_), len(indices_) * size_of(u32))
	return
}
