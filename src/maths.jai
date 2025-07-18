/*
* Copyright (c) 2025 Chris Giles
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Chris Giles makes no representations about the suitability
* of this software for any purpose.
* It is provided "as is" without express or implied warranty.
*/
sign :: inline (x: float) -> float
{
    return ifx x < 0 then -1.0 else (ifx x > 0 then 1.0 else 0.0);
}

xy :: inline (in: Vector3) -> Vector2 {
    return .{in.x, in.y};
}

cross :: inline ( a: Vector2,  b: Vector2) -> float32
{
    return a.x * b.y - a.y * b.x;
}

rotation :: inline ( angle: float32) -> Matrix2
{
     c := cos(angle);
     s := sin(angle);
    return .{ c, -s, s, c };
}

 transform :: inline ( q: Vector3, v: Vector2) -> Vector2
{
    return rotation(q.z) * v + xy(q);
}

  outer :: inline ( a: Vector2,  b: Vector2) -> Matrix2
{
     m: Matrix2; 
     m.v = .[ b * a.x, b * a.y ];
    return m;
}

  outer :: inline ( a: Vector3,  b: Vector3) -> Matrix3
{
     m: Matrix3; 
     m.v = .[ b * a.x, b * a.y, b * a.z ];
    return m;
}

transpose :: (m: *Matrix2) {
    Swap(*m.coef[0][1], *m.coef[1][0]);
}

transpose :: (mconst: Matrix2) -> Matrix2 {
    m := mconst;
    Swap(*m.coef[0][1], *m.coef[1][0]);
    return m;
}

abs :: inline (a: Matrix2) -> Matrix2
{   
    m: Matrix2 = ---;
    m.v[0] = abs(a.v[0]);
    m.v[1] = abs(a.v[1]);
    return m;
}

column :: (m: Matrix2, i: int) -> Vector2 {
    return .{m.coef[0][i], m.coef[1][i]};
}

column :: (m: Matrix3, i: int) -> Vector3 {
    return .{m.coef[0][i], m.coef[1][i], m.coef[2][i]};
}

diagonal:: inline (m00: float, m11: float, m22: float) -> Matrix3
{
    return Matrix3.{
        m00, 0, 0, 
        0, m11, 0,
        0, 0, m22
    };
}


solve :: inline(a : Matrix3, b: Vector3) -> Vector3
{
    // Compute LDL^T decomposition
    D1  := a.coef[0][0];
    L21 := a.coef[1][0] / a.coef[0][0];
    L31 := a.coef[2][0] / a.coef[0][0];
    D2  := a.coef[1][1] - L21 * L21 * D1;
    L32 := (a.coef[2][1] - L21 * L31 * D1) / D2;
    D3  := a.coef[2][2] - (L31 * L31 * D1 + L32 * L32 * D2);

    // Forward substitution: Solve Ly = b
    y1  := b.x;
    y2  := b.y - L21 * y1;
    y3  := b.z - L31 * y1 - L32 * y2;

    // Diagonal solve: Solve Dz = y
    z1  := y1 / D1;
    z2  := y2 / D2;
    z3  := y3 / D3;

    // Backward substitution: Solve L^T x = z
    x: Vector3;
    x.component[2] = z3;
    x.component[1] = z2 - L32 * x.component[2];
    x.component[0] = z1 - L21 * x.component[1] - L31 * x.component[2];

    return x;
}

