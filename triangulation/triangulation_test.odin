package triangulation

import "../clipper"
import "core:math"
import "core:testing"
import "shared:utils_private"
import "shared:utils_private/fixed_bcd"

@(private = "file")
DEF_FRAC_DIGITS :: fixed_bcd.MAX_FRAC_DIGITS

@(private)
_make_square_path_bcd :: proc(
	x0, y0, size: fixed_bcd.BCD($FRAC_DIGITS),
	invent := false,
	allocator := context.allocator,
) -> [][2]fixed_bcd.BCD(FRAC_DIGITS) {
	result := utils_private.make_non_zeroed_slice([][2]fixed_bcd.BCD(FRAC_DIGITS), 4, allocator)
	two := fixed_bcd.init_const(2, 0, 0, FRAC_DIGITS)
	half := fixed_bcd.div(size, two)
	if !invent {
		result[0] = {fixed_bcd.add(x0, half), fixed_bcd.add(y0, half)}
		result[1] = {fixed_bcd.sub(x0, half), fixed_bcd.add(y0, half)}
		result[2] = {fixed_bcd.sub(x0, half), fixed_bcd.sub(y0, half)}
		result[3] = {fixed_bcd.add(x0, half), fixed_bcd.sub(y0, half)}
	} else {
		result[0] = {fixed_bcd.sub(x0, half), fixed_bcd.sub(y0, half)}
		result[1] = {fixed_bcd.add(x0, half), fixed_bcd.sub(y0, half)}
		result[2] = {fixed_bcd.add(x0, half), fixed_bcd.add(y0, half)}
		result[3] = {fixed_bcd.sub(x0, half), fixed_bcd.add(y0, half)}
	}
	return result
}

poly1 := [?][2]f64 {
	{6251161, 332856160},
	{840876097, 97496650},
	{976400933, 140787098},
	{330832885, 702363622},
	{524959570, 901562500},
	{283075095, 283198665},
	{682169472, 407971968},
	{341184383, 906937707},
	{885255988, 51653123},
	{679161444, 348752493},
	{110729587, 243797389},
	{175478881, 936371388},
	{884834543, 92623405},
	{830335767, 487305557},
	{381715781, 603651314},
	{429388870, 750813644},
	{183632134, 133019917},
	{748295100, 710325195},
	{736200816, 526977435},
	{265700863, 815231128},
	{267777137, 451565516},
	{932290823, 419938943},
	{881163203, 459777725},
	{46306602, 10129599},
	{52939203, 969104432},
	{15564105, 724992816},
	{826186121, 204403883},
	{168323587, 84596478},
	{330051681, 190436576},
	{910281595, 436345833},
	{579089233, 926825204},
	{409518567, 421262563},
	{907897616, 740612275},
	{943299290, 731351779},
	{220519408, 944234682},
	{397472466, 978974872},
	{478544665, 67011261},
	{492508035, 881036163},
	{869736187, 774199458},
	{738244055, 744934646},
	{744662274, 427823310},
	{841438346, 988766232},
	{614037581, 326952247},
	{1868663, 40207860},
	{308127932, 719137146},
	{258010101, 520371199},
	{418166295, 915065961},
	{49983486, 843699463},
	{526874162, 817456881},
	{41058475, 738741192},
	{727641385, 611946004},
	{338496075, 630157593},
	{691414735, 818968108},
	{49426629, 734590805},
	{149386829, 315107107},
	{537222333, 388854339},
	{79101039, 347162131},
	{576707064, 71330961},
	{712674406, 422581668},
	{929289005, 867002665},
	{913051643, 149224610},
	{65254363, 479593145},
	{694329570, 11130378},
	{913734201, 50414969},
	{654447184, 797671163},
	{130981529, 731710403},
	{331099632, 659944678},
	{619403370, 520436929},
	{19628661, 496649629},
	{61993195, 185722653},
	{714388595, 163372694},
	{615296901, 93286726},
	{830312146, 332917500},
	{994042869, 607637909},
	{784366896, 187042198},
	{200105950, 610383617},
	{826144101, 905199409},
	{24835788, 324705858},
	{277723420, 728522750},
	{630447729, 937469734},
	{221564719, 91059621},
	{548009742, 327404397},
	{227909712, 840292896},
	{542525953, 664345792},
	{875391387, 975232306},
	{829573197, 125234027},
	{332393412, 80824462},
	{137298543, 537715464},
	{439096431, 641313184},
	{203515829, 441692082},
	{205715688, 667575336},
	{416227233, 414575851},
	{838344120, 95970179},
	{976010983, 268810085},
	{183789536, 362685970},
	{490023328, 406886322},
	{357540544, 401985157},
	{70912036, 799416867},
	{587931344, 340081589},
	{500905973, 96873619},
}

@(test)
test_custom :: proc(t: ^testing.T) {
	poly2: [][2]fixed_bcd.BCD(DEF_FRAC_DIGITS) = utils_private.make_non_zeroed_slice(
		[][2]fixed_bcd.BCD(DEF_FRAC_DIGITS),
		len(poly1),
		context.allocator,
	)
	defer delete(poly2)

	for i in 0 ..< len(poly1) {
		poly2[i][0] = fixed_bcd.from_f64(DEF_FRAC_DIGITS, poly1[i][0] / 10000.0)
		poly2[i][1] = fixed_bcd.from_f64(DEF_FRAC_DIGITS, poly1[i][1] / 10000.0)
	}

	res, res_open, err := clipper.BooleanOp_Fixed(
		.Union,
		[][][2]fixed_bcd.BCD(DEF_FRAC_DIGITS){poly2[:]},
		nil,
		nil,
		.NonZero,
	)
	testing.expect_value(t, err, nil)

	indices, terr := TrianguatePolygons_Fixed(res)
	defer delete(indices)

	testing.expect_value(t, terr, nil)
	testing.expect_value(t, len(indices), 12)
}

@(test)
test_triangulation_2square :: proc(t: ^testing.T) {
	x0 := fixed_bcd.init_const(2, 0, 0, DEF_FRAC_DIGITS)
	y0 := fixed_bcd.init_const(0, 0, 0, DEF_FRAC_DIGITS)
	size := fixed_bcd.init_const(100, 0, 0, DEF_FRAC_DIGITS)

	x1 := fixed_bcd.init_const(25, 0, 0, DEF_FRAC_DIGITS)
	y1 := fixed_bcd.init_const(-25, 0, 0, DEF_FRAC_DIGITS)
	size2 := fixed_bcd.init_const(50, 0, 0, DEF_FRAC_DIGITS)

	square := _make_square_path_bcd(x0, y0, size)
	defer delete(square)

	square2 := _make_square_path_bcd(x1, y1, size2)
	defer delete(square2)

	poly := [][][2]fixed_bcd.BCD(DEF_FRAC_DIGITS){square, square2}
	indices, err := TrianguatePolygons_Fixed(poly)
	defer delete(indices)

	testing.expect_value(t, err, nil)
	testing.expect_value(t, len(indices), 12)
}

// @(test)
// test_cross_product_sign_convention :: proc(t: ^testing.T) {
// 	p0 := [2]fixed_bcd.BCD(DEF_FRAC_DIGITS) {
// 		fixed_bcd.init_const(0, 0, 0, DEF_FRAC_DIGITS),
// 		fixed_bcd.init_const(0, 0, 0, DEF_FRAC_DIGITS),
// 	}
// 	p1 := [2]fixed_bcd.BCD(DEF_FRAC_DIGITS) {
// 		fixed_bcd.init_const(1, 0, 0, DEF_FRAC_DIGITS),
// 		fixed_bcd.init_const(0, 0, 0, DEF_FRAC_DIGITS),
// 	}
// 	p2 := [2]fixed_bcd.BCD(DEF_FRAC_DIGITS) {
// 		fixed_bcd.init_const(1, 0, 0, DEF_FRAC_DIGITS),
// 		fixed_bcd.init_const(1, 0, 0, DEF_FRAC_DIGITS),
// 	}

// 	testing.expect_value(t, CrossProductSign(p0, p1, p2), 1)
// 	testing.expect_value(
// 		t,
// 		GetPolygonOrientation([][2]fixed_bcd.BCD(DEF_FRAC_DIGITS){p0, p1, p2}),
// 		PolyOrientation.CounterClockwise,
// 	)
// 	testing.expect_value(t, CrossProductSign(p0, p2, p1), -1)
// 	testing.expect_value(
// 		t,
// 		GetPolygonOrientation([][2]fixed_bcd.BCD(DEF_FRAC_DIGITS){p0, p2, p1}),
// 		PolyOrientation.Clockwise,
// 	)
// }
