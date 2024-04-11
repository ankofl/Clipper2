using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace Clipper2Lib
{
	public enum TPPLOrientation
	{
		TPPL_ORIENTATION_CW = -1,
		TPPL_ORIENTATION_NONE = 0,
		TPPL_ORIENTATION_CCW = 1,
	};

	public enum TPPLVertexType
	{
		TPPL_VERTEXTYPE_REGULAR = 0,
		TPPL_VERTEXTYPE_START = 1,
		TPPL_VERTEXTYPE_END = 2,
		TPPL_VERTEXTYPE_SPLIT = 3,
		TPPL_VERTEXTYPE_MERGE = 4,
	};

	// 2D point structure.
	public struct TPPLPoint
	{
		public double x;
		public double y;
		// User-specified vertex identifier. Note that this isn't used internally
		// by the library, but will be faithfully copied around.
		public int id;

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static TPPLPoint operator +(in TPPLPoint t, in TPPLPoint p)
		{
			TPPLPoint r;
			r.x = t.x + p.x;
			r.y = t.y + p.y;
			r.id = default;
			return r;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static TPPLPoint operator -(in TPPLPoint t, in TPPLPoint p)
		{
			TPPLPoint r;
			r.x = t.x - p.x;
			r.y = t.y - p.y;
			r.id = default;
			return r;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static TPPLPoint operator *(in TPPLPoint t, double f)
		{
			TPPLPoint r;
			r.x = t.x * f;
			r.y = t.y * f;
			r.id = default;
			return r;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static TPPLPoint operator /(in TPPLPoint t, double f)
		{
			TPPLPoint r;
			r.x = t.x / f;
			r.y = t.y / f;
			r.id = default;
			return r;
		}
	}

	// Polygon implemented as an array of points with a "hole" flag.
	public class TPPLPoly
	{
		protected TPPLPoint[] points;
		protected long numpoints;
		protected bool hole;

		// Constructors and destructors.
		public TPPLPoly()
		{
			hole = false;
			numpoints = 0;
			points = null;
		}

		public TPPLPoly(TPPLPoly src)
		{
			hole = src.hole;
			numpoints = src.numpoints;

			if (numpoints > 0)
			{
				points = new TPPLPoint[numpoints];
				Array.Copy(src.points, points, numpoints);
			}
		}

		public TPPLPoly CopyFrom(TPPLPoly src)
		{
			Clear();
			hole = src.hole;
			numpoints = src.numpoints;

			if (numpoints > 0)
			{
				points = new TPPLPoint[numpoints];
				Array.Copy(src.points, points, numpoints);
			}

			return this;
		}

		// Getters and setters.
		public long GetNumPoints()
		{
			return numpoints;
		}

		public bool IsHole()
		{
			return hole;
		}

		public void SetHole(bool hole)
		{
			this.hole = hole;
		}

		public TPPLPoint GetPoint(long i)
		{
			return points[i];
		}

		public TPPLPoint[] GetPoints()
		{
			return points;
		}

		public TPPLPoint this[long i]
		{
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			get => points[i];
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			set => points[i] = value;
		}

		// Clears the polygon points.
		public void Clear()
		{
			hole = false;
			numpoints = 0;
			points = null;
		}

		// Inits the polygon with numpoints vertices.
		public void Init(long numpoints)
		{
			Clear();
			this.numpoints = numpoints;
			points = new TPPLPoint[numpoints];
		}

		// Creates a triangle with points p1, p2, and p3.
		public void Triangle(in TPPLPoint p1, in TPPLPoint p2, in TPPLPoint p3)
		{
			Init(3);
			points[0] = p1;
			points[1] = p2;
			points[2] = p3;
		}

		// Inverts the orfer of vertices.
		public void Invert()
		{
			Array.Reverse(points);
		}

		// Returns the orientation of the polygon.
		// Possible values:
		//    TPPL_ORIENTATION_CCW: Polygon vertices are in counter-clockwise order.
		//    TPPL_ORIENTATION_CW: Polygon vertices are in clockwise order.
		//    TPPL_ORIENTATION_NONE: The polygon has no (measurable) area.
		public TPPLOrientation GetOrientation()
		{
			long i1, i2;
			double area = 0;
			for (i1 = 0; i1 < numpoints; i1++)
			{
				i2 = i1 + 1;
				if (i2 == numpoints)
				{
					i2 = 0;
				}
				area += points[i1].x * points[i2].y - points[i1].y * points[i2].x;
			}
			if (area > 0)
			{
				return TPPLOrientation.TPPL_ORIENTATION_CCW;
			}
			if (area < 0)
			{
				return TPPLOrientation.TPPL_ORIENTATION_CW;
			}
			return TPPLOrientation.TPPL_ORIENTATION_NONE;
		}

		// Sets the polygon orientation.
		// Possible values:
		//    TPPL_ORIENTATION_CCW: Sets vertices in counter-clockwise order.
		//    TPPL_ORIENTATION_CW: Sets vertices in clockwise order.
		//    TPPL_ORIENTATION_NONE: Reverses the orientation of the vertices if there
		//       is one, otherwise does nothing (if orientation is already NONE).
		public void SetOrientation(TPPLOrientation orientation)
		{
			TPPLOrientation polyorientation = GetOrientation();
			if (polyorientation != TPPLOrientation.TPPL_ORIENTATION_NONE && polyorientation != orientation)
			{
				Invert();
			}
		}

		// Checks whether a polygon is valid or not.
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public bool Valid()
		{ 
			return this.numpoints >= 3; 
		}
	};

	public class TPPLPartition
	{
		protected class PartitionVertex
		{
			public bool isActive;
			public bool isConvex;
			public bool isEar;

			public TPPLPoint p;
			public double angle;
			public PartitionVertex previous;
			public PartitionVertex next;

			public PartitionVertex()
			{
				previous = null;
				next = null;
			}
		};

		protected struct MonotoneVertex
		{
			public TPPLPoint p;
			public long previous;
			public long next;
		};

		protected class VertexSorter : IComparer<long>
		{
			MonotoneVertex[] vertices;

			public VertexSorter(MonotoneVertex[] v)
			{ vertices = v; }

			// Sorts in the falling order of y values, if y is equal, x is used instead.
			public int Compare(long index1, long index2)
			{
				if (vertices[index1].p.y > vertices[index2].p.y)
				{
					return -1;
				}
				else if (vertices[index1].p.y == vertices[index2].p.y)
				{
					if (vertices[index1].p.x > vertices[index2].p.x)
					{
						return -1;
					}
					else if (vertices[index1].p.x == vertices[index2].p.x)
					{
						return 0;
					}
				}
				return 1;
			}
		}
		protected struct Diagonal
		{
			public long index1;
			public long index2;
		};

		protected class DiagonalList : LinkedList<Diagonal> { };

		// Dynamic programming state for minimum-weight triangulation.
		protected struct DPState
		{
			public bool visible;
			public double weight;
			public long bestvertex;
		};

		// Dynamic programming state for convex partitioning.
		protected struct DPState2
		{
			public bool visible;
			public long weight;
			public DiagonalList pairs;
		};

		// Edge that intersects the scanline.
		protected struct ScanLineEdge
		{
			public long index;
			public TPPLPoint p1;
			public TPPLPoint p2;

			public static bool IsConvex(in TPPLPoint p1, in TPPLPoint p2, in TPPLPoint p3)
			{
				double tmp;
				tmp = (p3.y - p1.y) * (p2.x - p1.x) - (p3.x - p1.x) * (p2.y - p1.y);
				if (tmp > 0)
				{
					return true;
				}

				return false;
			}

			// Determines if the edge is to the left of another edge.
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			public static bool operator <(in ScanLineEdge t, in ScanLineEdge other)
			{
				if (other.p1.y == other.p2.y)
				{
					if (t.p1.y == t.p2.y)
					{
						return (t.p1.y < other.p1.y);
					}
					return IsConvex(t.p1, t.p2, other.p1);
				}
				else if (t.p1.y == t.p2.y)
				{
					return !IsConvex(other.p1, other.p2, t.p1);
				}
				else if (t.p1.y < other.p1.y)
				{
					return !IsConvex(other.p1, other.p2, t.p1);
				}
				else
				{
					return IsConvex(t.p1, t.p2, other.p1);
				}
			}
			// Determines if the edge is to the right of another edge.
			[MethodImpl(MethodImplOptions.AggressiveInlining)]
			public static bool operator >(in ScanLineEdge t, in ScanLineEdge other)
			{
				return other < t;
			}
		};

		// Standard helper functions.
		protected bool IsConvex(in TPPLPoint p1, in TPPLPoint p2, in TPPLPoint p3)
		{
			double tmp;
			tmp = (p3.y - p1.y) * (p2.x - p1.x) - (p3.x - p1.x) * (p2.y - p1.y);
			if (tmp > 0)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		protected bool IsReflex(in TPPLPoint p1, in TPPLPoint p2, in TPPLPoint p3)
		{
			double tmp;
			tmp = (p3.y - p1.y) * (p2.x - p1.x) - (p3.x - p1.x) * (p2.y - p1.y);
			if (tmp < 0)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		protected bool IsInside(in TPPLPoint p1, in TPPLPoint p2, in TPPLPoint p3, in TPPLPoint p)
		{
			if (IsConvex(p1, p, p2))
			{
				return false;
			}
			if (IsConvex(p2, p, p3))
			{
				return false;
			}
			if (IsConvex(p3, p, p1))
			{
				return false;
			}
			return true;
		}

		protected bool InCone(in TPPLPoint p1, in TPPLPoint p2, in TPPLPoint p3, in TPPLPoint p)
		{
			bool convex;

			convex = IsConvex(p1, p2, p3);

			if (convex)
			{
				if (!IsConvex(p1, p2, p))
				{
					return false;
				}
				if (!IsConvex(p2, p3, p))
				{
					return false;
				}
				return true;
			}
			else
			{
				if (IsConvex(p1, p2, p))
				{
					return true;
				}
				if (IsConvex(p2, p3, p))
				{
					return true;
				}
				return false;
			}
		}
		protected bool InCone(PartitionVertex v, in TPPLPoint p)
		{
			TPPLPoint p1, p2, p3;

			p1 = v.previous.p;
			p2 = v.p;
			p3 = v.next.p;

			return InCone(p1, p2, p3, p);
		}

		// Checks if two lines intersect.
		protected bool Intersects(in TPPLPoint p11, in TPPLPoint p12, in TPPLPoint p21, in TPPLPoint p22)
		{
			if ((p11.x == p21.x) && (p11.y == p21.y))
			{
				return false;
			}
			if ((p11.x == p22.x) && (p11.y == p22.y))
			{
				return false;
			}
			if ((p12.x == p21.x) && (p12.y == p21.y))
			{
				return false;
			}
			if ((p12.x == p22.x) && (p12.y == p22.y))
			{
				return false;
			}

			TPPLPoint v1ort, v2ort, v;
			double dot11, dot12, dot21, dot22;

			v1ort.x = p12.y - p11.y;
			v1ort.y = p11.x - p12.x;

			v2ort.x = p22.y - p21.y;
			v2ort.y = p21.x - p22.x;

			v = p21 - p11;
			dot21 = v.x * v1ort.x + v.y * v1ort.y;
			v = p22 - p11;
			dot22 = v.x * v1ort.x + v.y * v1ort.y;

			v = p11 - p21;
			dot11 = v.x * v2ort.x + v.y * v2ort.y;
			v = p12 - p21;
			dot12 = v.x * v2ort.x + v.y * v2ort.y;

			if (dot11 * dot12 > 0)
			{
				return false;
			}
			if (dot21 * dot22 > 0)
			{
				return false;
			}

			return true;
		}

		protected TPPLPoint Normalize(in TPPLPoint p)
		{
			TPPLPoint r;
			double n = Math.Sqrt(p.x * p.x + p.y * p.y);
			if (n != 0)
			{
				r = p / n;
			}
			else
			{
				r.x = 0;
				r.y = 0;
			}
			r.id = default;
			return r;
		}
		protected double Distance(in TPPLPoint p1, in TPPLPoint p2)
		{
			double dx, dy;
			dx = p2.x - p1.x;
			dy = p2.y - p1.y;
			return (Math.Sqrt(dx * dx + dy * dy));
		}

		// Helper functions for Triangulate_EC.
		protected void UpdateVertexReflexity(PartitionVertex v)
		{
			PartitionVertex v1 = null, v3 = null;
			v1 = v.previous;
			v3 = v.next;
			v.isConvex = !IsReflex(v1.p, v.p, v3.p);
		}
		protected void UpdateVertex(PartitionVertex v, PartitionVertex[] vertices, long numvertices)
		{
			long i;
			PartitionVertex v3 = null;
			TPPLPoint vec1, vec3;

			PartitionVertex v1 = v.previous;
			v3 = v.next;

			v.isConvex = IsConvex(v1.p, v.p, v3.p);

			vec1 = Normalize(v1.p - v.p);
			vec3 = Normalize(v3.p - v.p);
			v.angle = vec1.x * vec3.x + vec1.y * vec3.y;

			if (v.isConvex)
			{
				v.isEar = true;
				for (i = 0; i < numvertices; i++)
				{
					if ((vertices[i].p.x == v.p.x) && (vertices[i].p.y == v.p.y))
					{
						continue;
					}
					if ((vertices[i].p.x == v1.p.x) && (vertices[i].p.y == v1.p.y))
					{
						continue;
					}
					if ((vertices[i].p.x == v3.p.x) && (vertices[i].p.y == v3.p.y))
					{
						continue;
					}
					if (IsInside(v1.p, v.p, v3.p, vertices[i].p))
					{
						v.isEar = false;
						break;
					}
				}
			}
			else
			{
				v.isEar = false;
			}
		}

		// Helper functions for ConvexPartition_OPT.
		protected void UpdateState(long a, long b, long w, long i, long j, DPState2[][] dpstates)
		{
			Diagonal newdiagonal;
			DiagonalList pairs = null;
			long w2;

			w2 = dpstates[a][b].weight;
			if (w > w2)
			{
				return;
			}

			pairs = dpstates[a][b].pairs;
			newdiagonal.index1 = i;
			newdiagonal.index2 = j;

			if (w < w2)
			{
				pairs.Clear();
				pairs.AddFirst(newdiagonal);
				dpstates[a][b].weight = w;
			}
			else
			{
				if ((pairs.Count != 0) && (i <= pairs.First.Value.index1))
				{
					return;
				}
				while ((pairs.Count != 0) && (pairs.First.Value.index2 >= j))
				{
					pairs.RemoveFirst();
				}
				pairs.AddFirst(newdiagonal);
			}
		}
		protected void TypeA(long i, long j, long k, PartitionVertex[] vertices, DPState2[][] dpstates)
		{
			DiagonalList pairs = null;
			LinkedListNode<Diagonal> iter, lastiter;
			long top;
			long w;

			if (!dpstates[i][j].visible)
			{
				return;
			}
			top = j;
			w = dpstates[i][j].weight;
			if (k - j > 1)
			{
				if (!dpstates[j][k].visible)
				{
					return;
				}
				w += dpstates[j][k].weight + 1;
			}
			if (j - i > 1)
			{
				pairs = dpstates[i][j].pairs;
				iter = pairs.Last;
				lastiter = null;
				while (iter != null)
				{
					if (!IsReflex(vertices[iter.Value.index2].p, vertices[j].p, vertices[k].p))
					{
						lastiter = iter;
					}
					else
					{
						break;
					}
					iter = iter.Previous;
				}
				if (lastiter == null)
				{
					w++;
				}
				else
				{
					if (IsReflex(vertices[k].p, vertices[i].p, vertices[lastiter.Value.index1].p))
					{
						w++;
					}
					else
					{
						top = lastiter.Value.index1;
					}
				}
			}
			UpdateState(i, k, w, top, j, dpstates);
		}
		protected void TypeB(long i, long j, long k, PartitionVertex[] vertices, DPState2[][] dpstates)
		{
			DiagonalList pairs = null;
			LinkedListNode<Diagonal> iter, lastiter;
			long top;
			long w;

			if (!dpstates[j][k].visible)
			{
				return;
			}
			top = j;
			w = dpstates[j][k].weight;

			if (j - i > 1)
			{
				if (!dpstates[i][j].visible)
				{
					return;
				}
				w += dpstates[i][j].weight + 1;
			}
			if (k - j > 1)
			{
				pairs = dpstates[j][k].pairs;

				iter = pairs.First;
				if ((pairs.Count != 0) && (!IsReflex(vertices[i].p, vertices[j].p, vertices[iter.Value.index1].p)))
				{
					lastiter = iter;
					while (iter != null)
					{
						if (!IsReflex(vertices[i].p, vertices[j].p, vertices[iter.Value.index1].p))
						{
							lastiter = iter;
							iter = iter.Next;
						}
						else
						{
							break;
						}
					}
					if (IsReflex(vertices[lastiter.Value.index2].p, vertices[k].p, vertices[i].p))
					{
						w++;
					}
					else
					{
						top = lastiter.Value.index2;
					}
				}
				else
				{
					w++;
				}
			}
			UpdateState(i, k, w, j, top, dpstates);
		}

		// Helper functions for MonotonePartition.
		protected bool Below(in TPPLPoint p1, in TPPLPoint p2)
		{
			if (p1.y < p2.y)
			{
				return true;
			}
			else if (p1.y == p2.y)
			{
				if (p1.x < p2.x)
				{
					return true;
				}
			}
			return false;
		}

		// Removes holes from inpolys by merging them with non-holes.
		// Simple heuristic procedure for removing holes from a list of polygons.
		// It works by creating a diagonal from the right-most hole vertex
		// to some other visible vertex.
		// Time complexity: O(h*(n^2)), h is the # of holes, n is the # of vertices.
		// Space complexity: O(n)
		// params:
		//    inpolys:
		//       A list of polygons that can contain holes.
		//       Vertices of all non-hole polys have to be in counter-clockwise order.
		//       Vertices of all hole polys have to be in clockwise order.
		//    outpolys:
		//       A list of polygons without holes.
		// Returns 1 on success, 0 on failure.
		public bool RemoveHoles(LinkedList<TPPLPoly> inpolys, LinkedList<TPPLPoly> outpolys)
		{
			LinkedList<TPPLPoly> polys;
			LinkedListNode<TPPLPoly> holeiter = default, polyiter = default, iter, iter2;
			int i, i2, holepointindex = default, polypointindex = default;
			TPPLPoint holepoint, polypoint, bestpolypoint = default;
			TPPLPoint linep1, linep2;
			TPPLPoint v1, v2;
			TPPLPoly newpoly = new();
			bool hasholes;
			bool pointvisible;
			bool pointfound;

			// Check for the trivial case of no holes.
			hasholes = false;
			for (iter = inpolys.First; iter != null; iter = iter.Next)
			{
				if (iter.Value.IsHole())
				{
					hasholes = true;
					break;
				}
			}
			if (!hasholes)
			{
				for (iter = inpolys.First; iter != null; iter = iter.Next)
				{
					outpolys.AddLast(new TPPLPoly(iter.Value));
				}
				return true;
			}

			polys = new LinkedList<TPPLPoly>(inpolys);

			while (true)
			{
				// Find the hole point with the largest x.
				hasholes = false;
				for (iter = polys.First; iter != null; iter = iter.Next)
				{
					if (!iter.Value.IsHole())
					{
						continue;
					}

					if (!hasholes)
					{
						hasholes = true;
						holeiter = iter;
						holepointindex = 0;
					}

					for (i = 0; i < iter.Value.GetNumPoints(); i++)
					{
						if (iter.Value.GetPoint(i).x > holeiter.Value.GetPoint(holepointindex).x)
						{
							holeiter = iter;
							holepointindex = i;
						}
					}
				}
				if (!hasholes)
				{
					break;
				}
				holepoint = holeiter.Value.GetPoint(holepointindex);

				pointfound = false;
				for (iter = polys.First; iter != null; iter = iter.Next)
				{
					if (iter.Value.IsHole())
					{
						continue;
					}
					for (i = 0; i < iter.Value.GetNumPoints(); i++)
					{
						if (iter.Value.GetPoint(i).x <= holepoint.x)
						{
							continue;
						}
						if (!InCone(iter.Value.GetPoint((i + iter.Value.GetNumPoints() - 1) % (iter.Value.GetNumPoints())),
									iter.Value.GetPoint(i),
									iter.Value.GetPoint((i + 1) % (iter.Value.GetNumPoints())),
									holepoint))
						{
							continue;
						}
						polypoint = iter.Value.GetPoint(i);
						if (pointfound)
						{
							v1 = Normalize(polypoint - holepoint);
							v2 = Normalize(bestpolypoint - holepoint);
							if (v2.x > v1.x)
							{
								continue;
							}
						}
						pointvisible = true;
						for (iter2 = polys.First; iter2 != null; iter2 = iter2.Next)
						{
							if (iter2.Value.IsHole())
							{
								continue;
							}
							for (i2 = 0; i2 < iter2.Value.GetNumPoints(); i2++)
							{
								linep1 = iter2.Value.GetPoint(i2);
								linep2 = iter2.Value.GetPoint((i2 + 1) % (iter2.Value.GetNumPoints()));
								if (Intersects(holepoint, polypoint, linep1, linep2))
								{
									pointvisible = false;
									break;
								}
							}
							if (!pointvisible)
							{
								break;
							}
						}
						if (pointvisible)
						{
							pointfound = true;
							bestpolypoint = polypoint;
							polyiter = iter;
							polypointindex = i;
						}
					}
				}

				if (!pointfound)
				{
					return false;
				}

				newpoly.Init(holeiter.Value.GetNumPoints() + polyiter.Value.GetNumPoints() + 2);
				i2 = 0;
				for (i = 0; i <= polypointindex; i++)
				{
					newpoly[i2] = polyiter.Value.GetPoint(i);
					i2++;
				}
				for (i = 0; i <= holeiter.Value.GetNumPoints(); i++)
				{
					newpoly[i2] = holeiter.Value.GetPoint((i + holepointindex) % holeiter.Value.GetNumPoints());
					i2++;
				}
				for (i = polypointindex; i < polyiter.Value.GetNumPoints(); i++)
				{
					newpoly[i2] = polyiter.Value.GetPoint(i);
					i2++;
				}

				polys.Remove(holeiter);
				polys.Remove(polyiter);
				polys.AddLast(newpoly);
			}

			for (iter = polys.First; iter != null; iter = iter.Next)
			{
				outpolys.AddLast(iter.Value);
			}

			return true;
		}

		// Triangulates a polygon by ear clipping.
		// Time complexity: O(n^2), n is the number of vertices.
		// Space complexity: O(n)
		// params:
		//    poly:
		//       An input polygon to be triangulated.
		//       Vertices have to be in counter-clockwise order.
		//    triangles:
		//       A list of triangles (result).
		// Returns 1 on success, 0 on failure.
		public bool Triangulate_EC(TPPLPoly poly, LinkedList<TPPLPoly> triangles)
		{
			if (!poly.Valid())
			{
				return false;
			}

			long numvertices;
			PartitionVertex[] vertices = null;
			PartitionVertex ear = null;
			TPPLPoly triangle = new();
			long i, j;
			bool earfound;

			if (poly.GetNumPoints() < 3)
			{
				return false;
			}
			if (poly.GetNumPoints() == 3)
			{
				triangles.AddLast(new TPPLPoly(poly));
				return true;
			}

			numvertices = poly.GetNumPoints();

			vertices = new PartitionVertex[numvertices];
			////TODO убрать
			//vertices[numvertices - 1] = new PartitionVertex();
			////***
			for (i = 0; i < numvertices; i++)
			{
				//vertices[i] = new PartitionVertex();
				vertices[i].isActive = true;
				vertices[i].p = poly.GetPoint(i);
				if (i == (numvertices - 1))
				{
					vertices[i].next = vertices[0];
				}
				else
				{
					vertices[i].next = vertices[i + 1];
				}
				if (i == 0)
				{
					vertices[i].previous = vertices[numvertices - 1];
				}
				else
				{
					vertices[i].previous = vertices[i - 1];
				}
			}
			for (i = 0; i < numvertices; i++)
			{
				UpdateVertex(vertices[i], vertices, numvertices);
			}

			for (i = 0; i < numvertices - 3; i++)
			{
				earfound = false;
				// Find the most extruded ear.
				for (j = 0; j < numvertices; j++)
				{
					if (!vertices[j].isActive)
					{
						continue;
					}
					if (!vertices[j].isEar)
					{
						continue;
					}
					if (!earfound)
					{
						earfound = true;
						ear = vertices[j];
					}
					else
					{
						if (vertices[j].angle > ear.angle)
						{
							ear = vertices[j];
						}
					}
				}
				if (!earfound)
				{
					return false;
				}

				triangle.Triangle(ear.previous.p, ear.p, ear.next.p);
				triangles.AddLast(new TPPLPoly(triangle));

				ear.isActive = false;
				ear.previous.next = ear.next;
				ear.next.previous = ear.previous;

				if (i == numvertices - 4)
				{
					break;
				}

				UpdateVertex(ear.previous, vertices, numvertices);
				UpdateVertex(ear.next, vertices, numvertices);
			}
			for (i = 0; i < numvertices; i++)
			{
				if (vertices[i].isActive)
				{
					triangle.Triangle(vertices[i].previous.p, vertices[i].p, vertices[i].next.p);
					triangles.AddLast(new TPPLPoly(triangle));
					break;
				}
			}

			return true;
		}

		// Triangulates a list of polygons that may contain holes by ear clipping
		// algorithm. It first calls RemoveHoles to get rid of the holes, and then
		// calls Triangulate_EC for each resulting polygon.
		// Time complexity: O(h*(n^2)), h is the # of holes, n is the # of vertices.
		// Space complexity: O(n)
		// params:
		//    inpolys:
		//       A list of polygons to be triangulated (can contain holes).
		//       Vertices of all non-hole polys have to be in counter-clockwise order.
		//       Vertices of all hole polys have to be in clockwise order.
		//    triangles:
		//       A list of triangles (result).
		// Returns 1 on success, 0 on failure.
		public bool Triangulate_EC(LinkedList<TPPLPoly> inpolys, LinkedList<TPPLPoly> triangles)
		{
			LinkedList<TPPLPoly> outpolys = new();
			LinkedListNode<TPPLPoly> iter;

			if (!RemoveHoles(inpolys, outpolys))
			{
				return false;
			}
			for (iter = outpolys.First; iter != null; iter = iter.Next)
			{
				if (!Triangulate_EC(iter.Value, triangles))
				{
					return false;
				}
			}
			return true;
		}

		// Creates an optimal polygon triangulation in terms of minimal edge length.
		// Time complexity: O(n^3), n is the number of vertices
		// Space complexity: O(n^2)
		// params:
		//    poly:
		//       An input polygon to be triangulated.
		//       Vertices have to be in counter-clockwise order.
		//    triangles:
		//       A list of triangles (result).
		// Returns 1 on success, 0 on failure.
		public bool Triangulate_OPT(TPPLPoly poly, LinkedList<TPPLPoly> triangles)
		{
			if (!poly.Valid())
			{
				return false;
			}

			long i, j, k, gap, n;
			DPState[][] dpstates = null;
			TPPLPoint p1, p2, p3, p4;
			long bestvertex;
			double weight, minweight = default, d1, d2;
			Diagonal diagonal, newdiagonal;
			DiagonalList diagonals = new();
			TPPLPoly triangle = new();
			bool ret = true;

			n = poly.GetNumPoints();
			dpstates = new DPState[n][];
			for (i = 1; i < n; i++)
			{
				dpstates[i] = new DPState[i];
			}

			// Initialize states and visibility.
			for (i = 0; i < (n - 1); i++)
			{
				p1 = poly.GetPoint(i);
				for (j = i + 1; j < n; j++)
				{
					dpstates[j][i].visible = true;
					dpstates[j][i].weight = 0;
					dpstates[j][i].bestvertex = -1;
					if (j != (i + 1))
					{
						p2 = poly.GetPoint(j);

						// Visibility check.
						if (i == 0)
						{
							p3 = poly.GetPoint(n - 1);
						}
						else
						{
							p3 = poly.GetPoint(i - 1);
						}
						if (i == (n - 1))
						{
							p4 = poly.GetPoint(0);
						}
						else
						{
							p4 = poly.GetPoint(i + 1);
						}
						if (!InCone(p3, p1, p4, p2))
						{
							dpstates[j][i].visible = false;
							continue;
						}

						if (j == 0)
						{
							p3 = poly.GetPoint(n - 1);
						}
						else
						{
							p3 = poly.GetPoint(j - 1);
						}
						if (j == (n - 1))
						{
							p4 = poly.GetPoint(0);
						}
						else
						{
							p4 = poly.GetPoint(j + 1);
						}
						if (!InCone(p3, p2, p4, p1))
						{
							dpstates[j][i].visible = false;
							continue;
						}

						for (k = 0; k < n; k++)
						{
							p3 = poly.GetPoint(k);
							if (k == (n - 1))
							{
								p4 = poly.GetPoint(0);
							}
							else
							{
								p4 = poly.GetPoint(k + 1);
							}
							if (Intersects(p1, p2, p3, p4))
							{
								dpstates[j][i].visible = false;
								break;
							}
						}
					}
				}
			}
			dpstates[n - 1][0].visible = true;
			dpstates[n - 1][0].weight = 0;
			dpstates[n - 1][0].bestvertex = -1;

			for (gap = 2; gap < n; gap++)
			{
				for (i = 0; i < (n - gap); i++)
				{
					j = i + gap;
					if (!dpstates[j][i].visible)
					{
						continue;
					}
					bestvertex = -1;
					for (k = (i + 1); k < j; k++)
					{
						if (!dpstates[k][i].visible)
						{
							continue;
						}
						if (!dpstates[j][k].visible)
						{
							continue;
						}

						if (k <= (i + 1))
						{
							d1 = 0;
						}
						else
						{
							d1 = Distance(poly.GetPoint(i), poly.GetPoint(k));
						}
						if (j <= (k + 1))
						{
							d2 = 0;
						}
						else
						{
							d2 = Distance(poly.GetPoint(k), poly.GetPoint(j));
						}

						weight = dpstates[k][i].weight + dpstates[j][k].weight + d1 + d2;

						if ((bestvertex == -1) || (weight < minweight))
						{
							bestvertex = k;
							minweight = weight;
						}
					}
					if (bestvertex == -1)
					{
						return false;
					}

					dpstates[j][i].bestvertex = bestvertex;
					dpstates[j][i].weight = minweight;
				}
			}

			newdiagonal.index1 = 0;
			newdiagonal.index2 = n - 1;
			diagonals.AddLast(newdiagonal);
			while (diagonals.Count != 0)
			{
				diagonal = diagonals.First.Value;
				diagonals.RemoveFirst();
				bestvertex = dpstates[diagonal.index2][diagonal.index1].bestvertex;
				if (bestvertex == -1)
				{
					ret = false;
					break;
				}
				triangle.Triangle(poly.GetPoint(diagonal.index1), poly.GetPoint(bestvertex), poly.GetPoint(diagonal.index2));
				triangles.AddLast(new TPPLPoly(triangle));
				if (bestvertex > (diagonal.index1 + 1))
				{
					newdiagonal.index1 = diagonal.index1;
					newdiagonal.index2 = bestvertex;
					diagonals.AddLast(newdiagonal);
				}
				if (diagonal.index2 > (bestvertex + 1))
				{
					newdiagonal.index1 = bestvertex;
					newdiagonal.index2 = diagonal.index2;
					diagonals.AddLast(newdiagonal);
				}
			}

			return ret;
		}

		// Partitions a polygon into convex polygons by using the
		// Hertel-Mehlhorn algorithm. The algorithm gives at most four times
		// the number of parts as the optimal algorithm, however, in practice
		// it works much better than that and often gives optimal partition.
		// It uses triangulation obtained by ear clipping as intermediate result.
		// Time complexity O(n^2), n is the number of vertices.
		// Space complexity: O(n)
		// params:
		//    poly:
		//       An input polygon to be partitioned.
		//       Vertices have to be in counter-clockwise order.
		//    parts:
		//       Resulting list of convex polygons.
		// Returns 1 on success, 0 on failure.
		public bool ConvexPartition_HM(TPPLPoly poly, LinkedList<TPPLPoly> parts)
		{
			if (!poly.Valid())
			{
				return false;
			}

			LinkedList<TPPLPoly> triangles = new();
			LinkedListNode<TPPLPoly> iter1, iter2;
			TPPLPoly poly1 = null, poly2 = null;
			TPPLPoly newpoly = new();
			TPPLPoint d1, d2, p1, p2, p3;
			long i11, i12, i21 = default, i22 = default, i13, i23, j, k;
			bool isdiagonal;
			long numreflex;

			// Check if the poly is already convex.
			numreflex = 0;
			for (i11 = 0; i11 < poly.GetNumPoints(); i11++)
			{
				if (i11 == 0)
				{
					i12 = poly.GetNumPoints() - 1;
				}
				else
				{
					i12 = i11 - 1;
				}
				if (i11 == (poly.GetNumPoints() - 1))
				{
					i13 = 0;
				}
				else
				{
					i13 = i11 + 1;
				}
				if (IsReflex(poly.GetPoint(i12), poly.GetPoint(i11), poly.GetPoint(i13)))
				{
					numreflex = 1;
					break;
				}
			}
			if (numreflex == 0)
			{
				parts.AddLast(new TPPLPoly(poly));
				return true;
			}

			if (!Triangulate_EC(poly, triangles))
			{
				return false;
			}

			for (iter1 = triangles.First; iter1 != null; iter1 = iter1.Next)
			{
				poly1 = iter1.Value;
				for (i11 = 0; i11 < poly1.GetNumPoints(); i11++)
				{
					d1 = poly1.GetPoint(i11);
					i12 = (i11 + 1) % (poly1.GetNumPoints());
					d2 = poly1.GetPoint(i12);

					isdiagonal = false;
					for (iter2 = iter1; iter2 != null; iter2 = iter2.Next)
					{
						if (iter1 == iter2)
						{
							continue;
						}
						poly2 = iter2.Value;

						for (i21 = 0; i21 < poly2.GetNumPoints(); i21++)
						{
							if ((d2.x != poly2.GetPoint(i21).x) || (d2.y != poly2.GetPoint(i21).y))
							{
								continue;
							}
							i22 = (i21 + 1) % (poly2.GetNumPoints());
							if ((d1.x != poly2.GetPoint(i22).x) || (d1.y != poly2.GetPoint(i22).y))
							{
								continue;
							}
							isdiagonal = true;
							break;
						}
						if (isdiagonal)
						{
							break;
						}
					}

					if (!isdiagonal)
					{
						continue;
					}

					p2 = poly1.GetPoint(i11);
					if (i11 == 0)
					{
						i13 = poly1.GetNumPoints() - 1;
					}
					else
					{
						i13 = i11 - 1;
					}
					p1 = poly1.GetPoint(i13);
					if (i22 == (poly2.GetNumPoints() - 1))
					{
						i23 = 0;
					}
					else
					{
						i23 = i22 + 1;
					}
					p3 = poly2.GetPoint(i23);

					if (!IsConvex(p1, p2, p3))
					{
						continue;
					}

					p2 = poly1.GetPoint(i12);
					if (i12 == (poly1.GetNumPoints() - 1))
					{
						i13 = 0;
					}
					else
					{
						i13 = i12 + 1;
					}
					p3 = poly1.GetPoint(i13);
					if (i21 == 0)
					{
						i23 = poly2.GetNumPoints() - 1;
					}
					else
					{
						i23 = i21 - 1;
					}
					p1 = poly2.GetPoint(i23);

					if (!IsConvex(p1, p2, p3))
					{
						continue;
					}

					newpoly.Init(poly1.GetNumPoints() + poly2.GetNumPoints() - 2);
					k = 0;
					for (j = i12; j != i11; j = (j + 1) % (poly1.GetNumPoints()))
					{
						newpoly[k] = poly1.GetPoint(j);
						k++;
					}
					for (j = i22; j != i21; j = (j + 1) % (poly2.GetNumPoints()))
					{
						newpoly[k] = poly2.GetPoint(j);
						k++;
					}

					triangles.Remove(iter2);
					iter1.Value.CopyFrom(newpoly);
					poly1 = iter1.Value;
					i11 = -1;

					continue;
				}
			}

			for (iter1 = triangles.First; iter1 != null; iter1 = iter1.Next)
			{
				parts.AddLast(new TPPLPoly(iter1.Value));
			}

			return true;
		}

		// Partitions a list of polygons into convex parts by using the
		// Hertel-Mehlhorn algorithm. The algorithm gives at most four times
		// the number of parts as the optimal algorithm, however, in practice
		// it works much better than that and often gives optimal partition.
		// It uses triangulation obtained by ear clipping as intermediate result.
		// Time complexity O(n^2), n is the number of vertices.
		// Space complexity: O(n)
		// params:
		//    inpolys:
		//       An input list of polygons to be partitioned. Vertices of
		//       all non-hole polys have to be in counter-clockwise order.
		//       Vertices of all hole polys have to be in clockwise order.
		//    parts:
		//       Resulting list of convex polygons.
		// Returns 1 on success, 0 on failure.
		public bool ConvexPartition_HM(LinkedList<TPPLPoly> inpolys, LinkedList<TPPLPoly> parts)
		{
			LinkedList<TPPLPoly> outpolys = new();
			LinkedListNode<TPPLPoly> iter;

			if (!RemoveHoles(inpolys, outpolys))
			{
				return false;
			}
			for (iter = outpolys.First; iter != null; iter = iter.Next)
			{
				if (!ConvexPartition_HM(iter.Value, parts))
				{
					return false;
				}
			}
			return true;
		}

		// Optimal convex partitioning (in terms of number of resulting
		// convex polygons) using the Keil-Snoeyink algorithm.
		// For reference, see M. Keil, J. Snoeyink, "On the time bound for
		// convex decomposition of simple polygons", 1998.
		// Time complexity O(n^3), n is the number of vertices.
		// Space complexity: O(n^3)
		// params:
		//    poly:
		//       An input polygon to be partitioned.
		//       Vertices have to be in counter-clockwise order.
		//    parts:
		//       Resulting list of convex polygons.
		// Returns 1 on success, 0 on failure.
		public bool ConvexPartition_OPT(TPPLPoly poly, LinkedList<TPPLPoly> parts)
		{
			if (!poly.Valid())
			{
				return false;
			}

			TPPLPoint p1, p2, p3, p4;
			PartitionVertex[] vertices = null;
			DPState2[][] dpstates = null;
			long i, j, k, n, gap;
			DiagonalList diagonals = new(), diagonals2 = new();
			Diagonal diagonal, newdiagonal;
			DiagonalList pairs = null, pairs2 = null;
			LinkedListNode<Diagonal> iter, iter2;
			bool ret;
			TPPLPoly newpoly = new();
			List<long> indices = new();
			bool ijreal, jkreal;

			n = poly.GetNumPoints();
			vertices = new PartitionVertex[n];

			dpstates = new DPState2[n][];
			for (i = 0; i < n; i++)
			{
				dpstates[i] = new DPState2[n];
				for (j = 0; j < n; j++)
				{
					dpstates[i][j].pairs = new();
				}
			}

			// Initialize vertex information.
			for (i = 0; i < n; i++)
			{
				vertices[i].p = poly.GetPoint(i);
				vertices[i].isActive = true;
				if (i == 0)
				{
					vertices[i].previous = vertices[n - 1];
				}
				else
				{
					vertices[i].previous = vertices[i - 1];
				}
				if (i == (poly.GetNumPoints() - 1))
				{
					vertices[i].next = vertices[0];
				}
				else
				{
					vertices[i].next = vertices[i + 1];
				}
			}
			for (i = 1; i < n; i++)
			{
				UpdateVertexReflexity(vertices[i]);
			}

			// Initialize states and visibility.
			for (i = 0; i < (n - 1); i++)
			{
				p1 = poly.GetPoint(i);
				for (j = i + 1; j < n; j++)
				{
					dpstates[i][j].visible = true;
					if (j == i + 1)
					{
						dpstates[i][j].weight = 0;
					}
					else
					{
						dpstates[i][j].weight = 2147483647;
					}
					if (j != (i + 1))
					{
						p2 = poly.GetPoint(j);

						// Visibility check.
						if (!InCone(vertices[i], p2))
						{
							dpstates[i][j].visible = false;
							continue;
						}
						if (!InCone(vertices[j], p1))
						{
							dpstates[i][j].visible = false;
							continue;
						}

						for (k = 0; k < n; k++)
						{
							p3 = poly.GetPoint(k);
							if (k == (n - 1))
							{
								p4 = poly.GetPoint(0);
							}
							else
							{
								p4 = poly.GetPoint(k + 1);
							}
							if (Intersects(p1, p2, p3, p4))
							{
								dpstates[i][j].visible = false;
								break;
							}
						}
					}
				}
			}
			for (i = 0; i < (n - 2); i++)
			{
				j = i + 2;
				if (dpstates[i][j].visible)
				{
					dpstates[i][j].weight = 0;
					newdiagonal.index1 = i + 1;
					newdiagonal.index2 = i + 1;
					dpstates[i][j].pairs.AddLast(newdiagonal);
				}
			}

			dpstates[0][n - 1].visible = true;
			vertices[0].isConvex = false; // By convention.

			for (gap = 3; gap < n; gap++)
			{
				for (i = 0; i < n - gap; i++)
				{
					if (vertices[i].isConvex)
					{
						continue;
					}
					k = i + gap;
					if (dpstates[i][k].visible)
					{
						if (!vertices[k].isConvex)
						{
							for (j = i + 1; j < k; j++)
							{
								TypeA(i, j, k, vertices, dpstates);
							}
						}
						else
						{
							for (j = i + 1; j < (k - 1); j++)
							{
								if (vertices[j].isConvex)
								{
									continue;
								}
								TypeA(i, j, k, vertices, dpstates);
							}
							TypeA(i, k - 1, k, vertices, dpstates);
						}
					}
				}
				for (k = gap; k < n; k++)
				{
					if (vertices[k].isConvex)
					{
						continue;
					}
					i = k - gap;
					if ((vertices[i].isConvex) && (dpstates[i][k].visible))
					{
						TypeB(i, i + 1, k, vertices, dpstates);
						for (j = i + 2; j < k; j++)
						{
							if (vertices[j].isConvex)
							{
								continue;
							}
							TypeB(i, j, k, vertices, dpstates);
						}
					}
				}
			}

			// Recover solution.
			ret = true;
			newdiagonal.index1 = 0;
			newdiagonal.index2 = n - 1;
			diagonals.AddLast(newdiagonal);
			while (diagonals.Count != 0)
			{
				diagonal = diagonals.First.Value;
				diagonals.RemoveFirst();
				if ((diagonal.index2 - diagonal.index1) <= 1)
				{
					continue;
				}
				pairs = dpstates[diagonal.index1][diagonal.index2].pairs;
				if (pairs.Count == 0)
				{
					ret = false;
					break;
				}
				if (!vertices[diagonal.index1].isConvex)
				{
					iter = pairs.Last;
					j = iter.Value.index2;
					newdiagonal.index1 = j;
					newdiagonal.index2 = diagonal.index2;
					diagonals.AddLast(newdiagonal);
					if ((j - diagonal.index1) > 1)
					{
						if (iter.Value.index1 != iter.Value.index2)
						{
							pairs2 = dpstates[diagonal.index1][j].pairs;
							while (true)
							{
								if (pairs2.Count == 0)
								{
									ret = false;
									break;
								}
								iter2 = pairs2.Last;
								if (iter.Value.index1 != iter2.Value.index1)
								{
									pairs2.RemoveLast();
								}
								else
								{
									break;
								}
							}
							if (!ret)
							{
								break;
							}
						}
						newdiagonal.index1 = diagonal.index1;
						newdiagonal.index2 = j;
						diagonals.AddFirst(newdiagonal);
					}
				}
				else
				{
					iter = pairs.First;
					j = iter.Value.index1;
					newdiagonal.index1 = diagonal.index1;
					newdiagonal.index2 = j;
					diagonals.AddFirst(newdiagonal);
					if ((diagonal.index2 - j) > 1)
					{
						if (iter.Value.index1 != iter.Value.index2)
						{
							pairs2 = dpstates[j][diagonal.index2].pairs;
							while (true)
							{
								if (pairs2.Count == 0)
								{
									ret = false;
									break;
								}
								iter2 = pairs2.First;
								if (iter.Value.index2 != iter2.Value.index2)
								{
									pairs2.RemoveFirst();
								}
								else
								{
									break;
								}
							}
							if (!ret)
							{
								break;
							}
						}
						newdiagonal.index1 = j;
						newdiagonal.index2 = diagonal.index2;
						diagonals.AddFirst(newdiagonal);
					}
				}
			}

			if (!ret)
			{
				return ret;
			}

			newdiagonal.index1 = 0;
			newdiagonal.index2 = n - 1;
			diagonals.AddFirst(newdiagonal);
			while (diagonals.Count != 0)
			{
				diagonal = diagonals.First.Value;
				diagonals.RemoveFirst();
				if ((diagonal.index2 - diagonal.index1) <= 1)
				{
					continue;
				}

				indices.Clear();
				diagonals2.Clear();
				indices.Add(diagonal.index1);
				indices.Add(diagonal.index2);
				diagonals2.AddFirst(diagonal);

				while (diagonals2.Count != 0)
				{
					diagonal = diagonals2.First.Value;
					diagonals2.RemoveFirst();
					if ((diagonal.index2 - diagonal.index1) <= 1)
					{
						continue;
					}
					ijreal = true;
					jkreal = true;
					pairs = dpstates[diagonal.index1][diagonal.index2].pairs;
					if (!vertices[diagonal.index1].isConvex)
					{
						iter = pairs.Last;
						j = iter.Value.index2;
						if (iter.Value.index1 != iter.Value.index2)
						{
							ijreal = false;
						}
					}
					else
					{
						iter = pairs.First;
						j = iter.Value.index1;
						if (iter.Value.index1 != iter.Value.index2)
						{
							jkreal = false;
						}
					}

					newdiagonal.index1 = diagonal.index1;
					newdiagonal.index2 = j;
					if (ijreal)
					{
						diagonals.AddLast(newdiagonal);
					}
					else
					{
						diagonals2.AddLast(newdiagonal);
					}

					newdiagonal.index1 = j;
					newdiagonal.index2 = diagonal.index2;
					if (jkreal)
					{
						diagonals.AddLast(newdiagonal);
					}
					else
					{
						diagonals2.AddLast(newdiagonal);
					}

					indices.Add(j);
				}

				indices.Sort();
				newpoly.Init(indices.Count);
				k = 0;
				foreach (var index in indices)
				{
					newpoly[k] = vertices[index].p;
					k++;
				}
				parts.AddLast(new TPPLPoly(newpoly));
			}

			return ret;
		}
	}
}
