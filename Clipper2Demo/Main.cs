/*******************************************************************************
* Author    :  Angus Johnson                                                   *
* Date      :  1 January 2023                                                  *
* Website   :  http://www.angusj.com                                           *
* Copyright :  Angus Johnson 2010-2023                                         *
* License   :  http://www.boost.org/LICENSE_1_0.txt                            *
*******************************************************************************/

using System.Reflection;
using System.IO;
using Clipper2Lib;
using System;
using System.Collections.Generic;
using System.Net.Http.Headers;

namespace Clipper2Demo
{
	public class Application
	{
		public static void Main()
		{
			Console.WriteLine("Start");
			//SquaresTest(true);
			//TrianglesTest(true);
			//DiamondsTest(true);

			TPPLPoint[] points =
			{
				new()
				{
					x = 0,
					y = 0
				},
				new()
				{
					x = 10,
					y = 0
				},
				new()
				{
					x = 10,
					y = 10
				},
				new()
				{
					x = 0,
					y = 10
				},
				new()
				{
					x = 5,
					y = 5
				},
				new()
				{
					x = 0,
					y = 0
				},
			};


			TPPLPoly poly = new TPPLPoly();
			poly.Init(points.Length);
			for (int i = 0; i < points.Length; i++)
			{
				poly[i] = points[i];
			}

			var orient = poly.GetOrientation();


			TPPLPartition triangulator = new TPPLPartition();

			LinkedList<TPPLPoly> triangles = new LinkedList<TPPLPoly>();

			bool result = triangulator.Triangulate_EC(poly, triangles);
			//bool result2 = triangulator.ConvexPartition_HM(poly, triangles);

			Console.WriteLine("End");
			Console.ReadKey();
		}

		public static Paths64 Polytree_Union(Paths64 subjects, FillRule fillrule)
		{
			// of course this function is inefficient, 
			// but it's purpose is simply to test polytrees.
			PolyTree64 polytree = new PolyTree64();
			Clipper.BooleanOp(ClipType.Union, subjects, null, polytree, fillrule);
			return Clipper.PolyTreeToPaths64(polytree);
		}

		public static void SquaresTest(bool test_polytree = false)
		{
			const int size = 10;
			const int w = 800, h = 600;
			FillRule fillrule = FillRule.NonZero;

			Path64 shape = Clipper.MakePath(new int[] { 0, 0, size, 0, size, size, 0, size });
			Paths64 subjects = new(), solution;
			Random rand = new();
			for (int i = 0; i < h / size; ++i)
			{
				for (int j = 0; j < w / size; ++j)
				{
					shape = Clipper.TranslatePath(shape, size, 0);
					if (rand.Next(5) != 0) subjects.Add(shape);
				}
				shape = Clipper.TranslatePath(shape, (-w / size) * size, size);
			}

			if (test_polytree)
				solution = Polytree_Union(subjects, fillrule);
			else
				solution = Clipper.Union(subjects, fillrule);
		}

		public static void TrianglesTest(bool test_polytree = false)
		{
			const int size = 10;
			const int w = 800, h = 600;
			FillRule fillrule = FillRule.NonZero;

			Path64 tri1 = Clipper.MakePath(new int[] { 0, 0, size * 2, 0, size, size * 2 });
			Path64 tri2 = Clipper.MakePath(new int[] { size * 2, 0, size, size * 2, size * 3, size * 2 });

			Paths64 subjects = new(), solution;
			Random rand = new();
			for (int i = 0; i < h / size / 2; ++i)
			{
				for (int j = 0; j < w / size / 2; ++j)
				{
					if (rand.Next(4) != 0) subjects.Add(tri1);
					if (rand.Next(4) != 0) subjects.Add(tri2);
					tri1 = Clipper.TranslatePath(tri1, size * 2, 0);
					tri2 = Clipper.TranslatePath(tri2, size * 2, 0);
				}
				tri1 = Clipper.TranslatePath(tri1, (-w / size) * size, size * 2);
				tri2 = Clipper.TranslatePath(tri2, (-w / size) * size, size * 2);
			}

			if (test_polytree)
				solution = Polytree_Union(subjects, fillrule);
			else
				solution = Clipper.Union(subjects, fillrule);
		}

		public static void DiamondsTest(bool test_polytree = false)
		{
			const int size = 10;
			const int w = 800, h = 600;
			FillRule fillrule = FillRule.NonZero;

			Path64 shape = Clipper.MakePath(new int[] { size, 0, size * 2, size, size, size * 2, 0, size });
			Paths64 subjects = new(), solution;
			Random rand = new();
			for (int i = 0; i < h / size / 2; ++i)
			{
				for (int j = 0; j < w / size; ++j)
				{
					if (rand.Next(7) != 0) subjects.Add(shape);
					if ((j & 1) == 0)
						shape = Clipper.TranslatePath(shape, size, size);
					else
						shape = Clipper.TranslatePath(shape, size, -size);
				}
				shape = Clipper.TranslatePath(shape, (-w / size) * size, size * 2);
			}

			if (test_polytree)
				solution = Polytree_Union(subjects, fillrule);
			else
				solution = Clipper.Union(subjects, fillrule);
		}

		public static Paths64 LoadPathsFromResource(string resourceName)
		{
			using Stream stream = Assembly.GetExecutingAssembly().
			  GetManifestResourceStream(resourceName);
			if (stream == null) return new Paths64();
			using BinaryReader reader = new(stream);
			int len = reader.ReadInt32();
			Paths64 result = new(len);
			for (int i = 0; i < len; i++)
			{
				int len2 = reader.ReadInt32();
				Path64 p = new(len2);
				for (int j = 0; j < len2; j++)
				{
					long X = reader.ReadInt64();
					long Y = reader.ReadInt64();
					p.Add(new Point64(X, Y));
				}
				result.Add(p);
			}
			return result;
		}

		public static void ClipTestPolys()
		{
			FillRule fillrule = FillRule.NonZero;
			Paths64 subject = LoadPathsFromResource("ConsoleDemo.subj.bin");
			Paths64 clip = LoadPathsFromResource("ConsoleDemo.clip.bin");
			Paths64 solution = Clipper.Intersect(subject, clip, fillrule);
		}


	} //end Application
} //namespace
