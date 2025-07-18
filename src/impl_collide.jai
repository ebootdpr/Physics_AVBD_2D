
// Copied and modified from box2D-lite: https://github.com/erincatto/box2d-lite

/*
MIT License

Copyright (c) 2019 Erin Catto

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// Box vertex and edge numbering:
//
//        ^ y
//        |
//        e1
//   v2 ------ v1
//    |        |
// e2 |        | e4  -. x
//    |        |
//   v3 ------ v4
//        e3

Axis :: enum
{
	FACE_A_X;
	FACE_A_Y;
	FACE_B_X;
	FACE_B_Y;
};

EdgeNumbers :: enum u8
{
	NO_EDGE :: 0;
	EDGE1;
	EDGE2;
	EDGE3;
	EDGE4;
};

ClipVertex :: struct
{
	// ClipVertex() { fp.value = 0; }  not needed
	v: Vector2;
	fp: Manifold.FeaturePair;
};



Flip :: (fp: *Manifold.FeaturePair)
{
	temp := fp.e.inEdge1;
	fp.e.inEdge1 = fp.e.inEdge2;
	fp.e.inEdge2 = temp;

	temp = fp.e.outEdge1;
	fp.e.outEdge1 = fp.e.outEdge2;
	fp.e.outEdge2 = temp;
}


ClipSegmentToLine :: ( vOut: []ClipVertex,  vIn:  []ClipVertex,
	normal: Vector2,  offset: float, clipEdge: EdgeNumbers) -> s32
{
	// Start with no output points
	numOut :s32= 0;

	// Calculate the distance of end points to the line
	distance0 := dot(normal, vIn[0].v) - offset;
	distance1 := dot(normal, vIn[1].v) - offset;

	// If the points are behind the plane
	if (distance0 <= 0.0) 
	{vOut[numOut] = vIn[0];
	numOut += 1;
	}
	if (distance1 <= 0.0){
		 vOut[numOut] = vIn[1];
	numOut += 1;
	}

	// If the points are on different sides of the plane
	if (distance0 * distance1 < 0.0)
	{
		// Find intersection point of edge and plane
		interp := distance0 / (distance0 - distance1);
		vOut[numOut].v = vIn[0].v + (vIn[1].v - vIn[0].v) * interp;
		if (distance0 > 0.0)
		{
			vOut[numOut].fp = vIn[0].fp;
			vOut[numOut].fp.e.inEdge1 = clipEdge;
			vOut[numOut].fp.e.inEdge2 = EdgeNumbers.NO_EDGE;
		}
		else
		{
			vOut[numOut].fp = vIn[1].fp;
			vOut[numOut].fp.e.outEdge1 = clipEdge;
			vOut[numOut].fp.e.outEdge2 = EdgeNumbers.NO_EDGE;
		}
		numOut += 1;
	}

	return numOut;
}

// static void ComputeIncidentEdge(ClipVertex c[2], const float2& h, const float2& pos,
// 	const float2x2& Rot, const float2& normal) 
//const Type & . is automatic in jai.
ComputeIncidentEdge :: ( c: []ClipVertex, h: Vector2, pos: Vector2,
	Rot: Matrix2, normal: Vector2)
{
	// The normal is from the reference box. Convert it
	// to the incident boxe's frame and flip sign.
	RotT := transpose(Rot);
	n := -(RotT * normal);
	nAbs := abs(n);

	if (nAbs.x > nAbs.y)
	{
		if (sign(n.x) > 0.0)
		{
			c[0].v = Vector2.{ h.x, -h.y };
			c[0].fp.e.inEdge2 = .EDGE3;
			c[0].fp.e.outEdge2 = .EDGE4;

			c[1].v = Vector2.{ h.x, h.y };
			c[1].fp.e.inEdge2 = .EDGE4;
			c[1].fp.e.outEdge2 = .EDGE1;
		}
		else
		{
			c[0].v = Vector2.{ -h.x, h.y };
			c[0].fp.e.inEdge2 = .EDGE1;
			c[0].fp.e.outEdge2 = .EDGE2;

			c[1].v = Vector2.{ -h.x, -h.y };
			c[1].fp.e.inEdge2 = .EDGE2;
			c[1].fp.e.outEdge2 = .EDGE3;
		}
	}
	else
	{
		if (sign(n.y) > 0.0)
		{
			c[0].v = Vector2.{ h.x, h.y };
			c[0].fp.e.inEdge2 = .EDGE4;
			c[0].fp.e.outEdge2 = .EDGE1;

			c[1].v = Vector2.{ -h.x, h.y };
			c[1].fp.e.inEdge2 = .EDGE1;
			c[1].fp.e.outEdge2 = .EDGE2;
		}
		else
		{
			c[0].v = Vector2.{ -h.x, -h.y };
			c[0].fp.e.inEdge2 = .EDGE2;
			c[0].fp.e.outEdge2 = .EDGE3;

			c[1].v = Vector2.{ h.x, -h.y };
			c[1].fp.e.inEdge2 = .EDGE3;
			c[1].fp.e.outEdge2 = .EDGE4;
		}
	}

	c[0].v = pos + Rot * c[0].v;
	c[1].v = pos + Rot * c[1].v;
}



// The normal points from A to B
manifold_collide :: ( bodyA: *Rigid, bodyB: *Rigid, contacts: []Manifold.Contact) ->  s32
{
	normal: Vector2;

	// Setup
	hA: Vector2 = bodyA.size * 0.5;
	hB: Vector2 = bodyB.size * 0.5;

	posA: Vector2 = xy(bodyA.position);
	posB: Vector2 = xy(bodyB.position);

	RotA: Matrix2 = rotation(bodyA.position.z);
	RotB: Matrix2 = rotation(bodyB.position.z);

	RotAT: Matrix2 = transpose(RotA);
	RotBT: Matrix2 = transpose(RotB);

	dp: Vector2 = posB - posA;
	dA: Vector2 = RotAT * dp;
	dB: Vector2 = RotBT * dp;

	C: Matrix2 = RotAT * RotB;
	absC: Matrix2 = abs(C);
	absCT: Matrix2 = transpose(absC);

	// Box A faces
	faceA: Vector2 = abs(dA) - hA - absC * hB;
	if (faceA.x > 0.0 || faceA.y > 0.0)
		return 0;

	// Box B faces
	faceB: Vector2 = abs(dB) - absCT * hA - hB;
	if (faceB.x > 0.0 || faceB.y > 0.0)
		return 0;

	// Find best axis
	axis: Axis;
	separation: float;

	// Box A faces
	axis = Axis.FACE_A_X;
	separation = faceA.x;
	if (dA.x > 0.0) normal =column(RotA, 0);
	else normal = column(-1*RotA, 0);

	relativeTol: float : 0.95;
	absoluteTol: float : 0.01;

	if (faceA.y > relativeTol * separation + absoluteTol * hA.y)
	{
		axis = Axis.FACE_A_Y;
		separation = faceA.y;
		if (dA.y > 0.0) normal = column(RotA, 1);
		else normal = column(-1*RotA, 1);
	}

	// Box B faces
	if (faceB.x > relativeTol * separation + absoluteTol * hB.x)
	{
		axis = Axis.FACE_B_X;
		separation = faceB.x;
		if (dB.x > 0.0) normal = column(RotB, 0);
		else normal = column(-1*RotB, 0);
	}

	if (faceB.y > relativeTol * separation + absoluteTol * hB.y)
	{
		axis = Axis.FACE_B_Y;
		separation = faceB.y;
		if (dB.y > 0.0) normal = column(RotB, 1);
		else normal = column(-1*RotB, 1);
	}

	// Setup clipping plane data based on the separating axis
	frontNormal: Vector2; 
	sideNormal: Vector2;
	incidentEdge: [2]ClipVertex;
	front, negSide, posSide: float;
	negEdge: EdgeNumbers;
	posEdge: EdgeNumbers;

	// Compute the clipping lines and the line segment to be clipped.
	if axis ==	{
	case Axis.FACE_A_X;
	{
		frontNormal = normal;
		front = dot(posA, frontNormal) + hA.x;
		sideNormal =column( RotA, 1);
		side: float = dot(posA, sideNormal);
		negSide = -side + hA.y;
		posSide = side + hA.y;
		negEdge = EdgeNumbers.EDGE3;
		posEdge = EdgeNumbers.EDGE1;
		ComputeIncidentEdge(incidentEdge, hB, posB, RotB, frontNormal);
	}

	case Axis.FACE_A_Y;
	{
		frontNormal = normal;
		front = dot(posA, frontNormal) + hA.y;
		sideNormal =column( RotA, 0);
		side: float = dot(posA, sideNormal);
		negSide = -side + hA.x;
		posSide = side + hA.x;
		negEdge = EdgeNumbers.EDGE2;
		posEdge = EdgeNumbers.EDGE4;
		ComputeIncidentEdge(incidentEdge, hB, posB, RotB, frontNormal);
	}

	case Axis.FACE_B_X;
	{
		frontNormal = -normal;
		front = dot(posB, frontNormal) + hB.x;
		sideNormal =column( RotB, 1);
		side: float = dot(posB, sideNormal);
		negSide = -side + hB.y;
		posSide = side + hB.y;
		negEdge = EdgeNumbers.EDGE3;
		posEdge = EdgeNumbers.EDGE1;
		ComputeIncidentEdge(incidentEdge, hA, posA, RotA, frontNormal);
	}

	case Axis.FACE_B_Y;
	{
		frontNormal = -normal;
		front = dot(posB, frontNormal) + hB.y;
		sideNormal =column( RotB, 0);
		side : float = dot(posB, sideNormal);
		negSide = -side + hB.x;
		posSide = side + hB.x;
		negEdge = EdgeNumbers.EDGE2;
		posEdge = EdgeNumbers.EDGE4;
		ComputeIncidentEdge(incidentEdge, hA, posA, RotA, frontNormal);
	}
	}

	// clip other face with 5 box planes (1 face plane, 4 edge planes)

	clipPoints1: [2]ClipVertex;
	clipPoints2: [2]ClipVertex;
	np: s32;

	// Clip to box side 1
	np = ClipSegmentToLine(clipPoints1, incidentEdge, -sideNormal, negSide, negEdge);

	if (np < 2)
		return 0;

	// Clip to negative box side 1
	np = ClipSegmentToLine(clipPoints2, clipPoints1, sideNormal, posSide, posEdge);

	if (np < 2)
		return 0;

	// Now clipPoints2 contains the clipping points.
	// Due to roundoff, it is possible that clipping removes all points.

	numContacts: s32= 0;
	for i: 0 ..2-1
	{
		separation := dot(frontNormal, clipPoints2[i].v) - front;

		if (separation <= 0)
		{
			contacts[numContacts].normal = -normal;

			// slide contact point onto reference face (easy to cull)
			contacts[numContacts].rA = transpose(RotA) * (clipPoints2[i].v - frontNormal * separation - posA);
			contacts[numContacts].rB = transpose(RotB) * (clipPoints2[i].v - posB);
			contacts[numContacts].feature = clipPoints2[i].fp;

			if (axis == Axis.FACE_B_X || axis == Axis.FACE_B_Y)
			{
				Flip(*contacts[numContacts].feature);
				contacts[numContacts].rA = transpose(RotA) * (clipPoints2[i].v - posA);
				contacts[numContacts].rB = transpose(RotB) * (clipPoints2[i].v - frontNormal * separation - posB);
			}
			numContacts += 1;
			
		}
	}
	return numContacts;
}
