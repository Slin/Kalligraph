//
//  KGMeshGeneratorLoopBlinnOutline.cpp
//  Kalligraph
//

#include "KGMeshGeneratorLoopBlinn.h"
#include <stdlib.h>

namespace KG
{
	class LoopBlinnOutlineBuilder
	{
		using Status = MeshGeneratorLoopBlinn::Status;

		struct Edge
		{
			Vector2 start;
			Vector2 end;
			bool stroke;
		};
		struct Vertex
		{
			Vector2 point;
			std::vector<size_t> edges;
		};

	public:
		static Status Build(const PathCollection &paths, double width, TriangleMesh &mesh)
		{
			Status status;
			if(!std::isfinite(width))
			{
				return Status::InvalidArguments;
			}
			if(width == 0.0)
			{
				mesh = MeshGeneratorLoopBlinn::GetMeshForPathCollection(paths);
				return Status::Success;
			}
			if(std::abs(width) > 1.0e12)
			{
				return Status::InvalidArguments;
			}
			const double tolerance = std::abs(width) / 32.0;
			if(tolerance == 0.0)
			{
				return Status::SubdivisionLimit;
			}

			std::vector<Edge> original;
			PathCollection originalCurves;
			double magnitude = std::max(1.0, std::abs(width));
			for(const Path &path : paths.paths)
			{
				std::vector<Vector2> points;
				Path analyticPath;
				for(const PathSegment &segment : path.segments)
				{
					if(segment.type == PathSegment::TypePoint)
					{
						continue;
					}

					if((status = ValidateSegment(segment, 4, magnitude)) != Status::Success)
					{
						return status;
					}
					if(points.empty())
					{
						points.push_back(segment.controlPoints.front());
					}

					if((status = FlattenCurve(segment.controlPoints, tolerance * 0.25, 0, points)) != Status::Success)
					{
						return status;
					}
					if(segment.type == PathSegment::TypeBezierCubic)
					{
						if((status = AddQuadraticSegmentsForCubic(segment.controlPoints, tolerance * 0.25, 0, analyticPath)) != Status::Success)
						{
							return status;
						}
					}
					else
					{
						analyticPath.segments.push_back(segment);
					}
				}

				if((status = AddPolygon(points, false, original)) != Status::Success)
				{
					return status;
				}
				if(!analyticPath.segments.empty())
				{
					originalCurves.paths.push_back(analyticPath);
				}
			}

			const double epsilon = magnitude * std::numeric_limits<double>::epsilon() * 128.0;
			if(!std::isfinite(epsilon) || magnitude > 1.0e12)
			{
				return Status::InvalidArguments;
			}
			// Remove canceled edges before generating the stroke.
			std::vector<Edge> boundaries;
			if((status = GetBoundaryEdges(original, epsilon, boundaries)) != Status::Success)
			{
				return status;
			}
			std::vector<Edge> edges = boundaries;
			if((status = AddStroke(boundaries, std::abs(width), tolerance * 0.25, epsilon, edges)) != Status::Success)
			{
				return status;
			}
			std::vector<Edge> offsetEdges;
			if((status = GetBoundaryEdges(edges, epsilon, offsetEdges, true, width > 0.0)) != Status::Success)
			{
				return status;
			}
			PathCollection offsetCurves;
			if((status = FitContours(offsetEdges, tolerance * 0.5, epsilon, offsetCurves)) != Status::Success)
			{
				return status;
			}
			return width > 0.0 ? GenerateAnalyticMesh(offsetCurves, originalCurves, epsilon, mesh) : GenerateAnalyticMesh(originalCurves, offsetCurves, epsilon, mesh);
		}

		static Status Build(const PathCollection &silhouette, const PathCollection &fill, TriangleMesh &mesh)
		{
			Status status;
			double magnitude = 1.0;
			for(const PathCollection *paths : {&silhouette, &fill})
			{
				for(const Path &path : paths->paths)
				{
					for(const PathSegment &segment : path.segments)
					{
						if((status = ValidateSegment(segment, 3, magnitude)) != Status::Success)
						{
							return status;
						}
					}
				}
			}

			return GenerateAnalyticMesh(silhouette, fill, magnitude * std::numeric_limits<double>::epsilon() * 128.0, mesh);
		}

	private:
		static constexpr size_t MaximumEdges = 4096;
		static constexpr size_t MaximumCurvePoints = 1024;

		static Status ValidateSegment(const PathSegment &segment, size_t maximumControlPoints, double &magnitude)
		{
			size_t count = 0;
			switch(segment.type)
			{
				case PathSegment::TypeLine:
					count = 2;
					break;
				case PathSegment::TypeBezierQuadratic:
					count = 3;
					break;
				case PathSegment::TypeBezierCubic:
					count = 4;
					break;
				default:
					break;
			}

			if(!count || count > maximumControlPoints || segment.controlPoints.size() != count)
			{
				return Status::UnsupportedSegment;
			}
			for(const Vector2 &point : segment.controlPoints)
			{
				if(!std::isfinite(point.x) || !std::isfinite(point.y) || std::abs(point.x) > 1.0e12 || std::abs(point.y) > 1.0e12)
				{
					return Status::InvalidArguments;
				}
				magnitude = std::max(magnitude, std::max(std::abs(point.x), std::abs(point.y)));
			}
			return Status::Success;
		}

		static double GetCrossProduct(Vector2 a, Vector2 b)
		{
			return a.x * b.y - a.y * b.x;
		}

		static Vector2 GetDifference(Vector2 a, Vector2 b)
		{
			return {a.x - b.x, a.y - b.y};
		}

		static Vector2 GetPointOnEdge(const Edge &edge, double t)
		{
			return {edge.start.x + (edge.end.x - edge.start.x) * t, edge.start.y + (edge.end.y - edge.start.y) * t};
		}

		static bool ArePointsEqual(Vector2 a, Vector2 b, double epsilon)
		{
			return std::abs(a.x - b.x) <= epsilon && std::abs(a.y - b.y) <= epsilon;
		}

		static double GetEdgeXAtY(const Edge &edge, double y)
		{
			return edge.start.x + (y - edge.start.y) * (edge.end.x - edge.start.x) / (edge.end.y - edge.start.y);
		}

		static int CompareDouble(const void *a, const void *b)
		{
			const double x = *static_cast<const double *>(a);
			const double y = *static_cast<const double *>(b);
			if(x < y)
			{
				return -1;
			}
			if(x > y)
			{
				return 1;
			}
			return 0;
		}

		static void SortUniqueValues(std::vector<double> &values, double epsilon)
		{
			if(values.empty())
			{
				return;
			}
			qsort(values.data(), values.size(), sizeof(double), CompareDouble);
			size_t count = 1;
			for(size_t i = 1; i < values.size(); i++)
			{
				if(values[i] - values[count - 1] > epsilon)
				{
					values[count++] = values[i];
				}
			}
			values.resize(count);
		}

		static void SubdivideControlPoints(const std::vector<Vector2> &control, std::vector<Vector2> &left, std::vector<Vector2> &right)
		{
			std::vector<Vector2> work = control;
			left.clear();
			right.resize(control.size());
			for(size_t level = control.size(); level > 0; level--)
			{
				left.push_back(work.front());
				right[level - 1] = work[level - 1];
				for(size_t i = 0; i + 1 < level; i++)
				{
					work[i] = {(work[i].x + work[i + 1].x) * 0.5, (work[i].y + work[i + 1].y) * 0.5};
				}
			}
		}

		static Status FlattenCurve(const std::vector<Vector2> &control, double tolerance, unsigned depth, std::vector<Vector2> &points)
		{
			Status status;
			const Vector2 a = control.front();
			const Vector2 b = control.back();
			const double dx = b.x - a.x;
			const double dy = b.y - a.y;
			const double lengthSquared = dx * dx + dy * dy;
			double errorSquared = 0.0;
			for(size_t i = 1; i + 1 < control.size(); i++)
			{
				// Distance to the segment catches collinear curves that double back.
				double t = lengthSquared > 0.0 ? ((control[i].x - a.x) * dx + (control[i].y - a.y) * dy) / lengthSquared : 0.0;
				t = std::max(0.0, std::min(1.0, t));
				const double ex = control[i].x - a.x - t * dx;
				const double ey = control[i].y - a.y - t * dy;
				errorSquared = std::max(errorSquared, ex * ex + ey * ey);
			}
			if(errorSquared <= tolerance * tolerance)
			{
				if(points.size() >= MaximumCurvePoints)
				{
					return Status::SubdivisionLimit;
				}
				points.push_back(b);
				return Status::Success;
			}
			if(depth == 24)
			{
				return Status::SubdivisionLimit;
			}

			std::vector<Vector2> left;
			std::vector<Vector2> right;
			SubdivideControlPoints(control, left, right);
			if((status = FlattenCurve(left, tolerance, depth + 1, points)) != Status::Success)
			{
				return status;
			}
			if((status = FlattenCurve(right, tolerance, depth + 1, points)) != Status::Success)
			{
				return status;
			}
			return Status::Success;
		}

		static Status AddPolygon(const std::vector<Vector2> &points, bool stroke, std::vector<Edge> &edges)
		{
			if(points.size() < 3)
			{
				return Status::Success;
			}
			if(edges.size() + points.size() > MaximumEdges)
			{
				return Status::SubdivisionLimit;
			}
			for(size_t i = 0; i < points.size(); i++)
			{
				if(!ArePointsEqual(points[i], points[(i + 1) % points.size()], 0.0))
				{
					edges.push_back({points[i], points[(i + 1) % points.size()], stroke});
				}
			}
			return Status::Success;
		}

		static bool GetEdgeIntersection(const Edge &a, const Edge &b, double &t, double &u)
		{
			const Vector2 d = GetDifference(a.end, a.start);
			const Vector2 e = GetDifference(b.end, b.start);
			const Vector2 delta = GetDifference(b.start, a.start);
			const double divisor = GetCrossProduct(d, e);
			if(std::abs(divisor) <= std::numeric_limits<double>::epsilon() * 16.0 * (std::abs(d.x * e.y) + std::abs(d.y * e.x)))
			{
				return false;
			}
			t = GetCrossProduct(delta, e) / divisor;
			u = GetCrossProduct(delta, d) / divisor;
			return t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0;
		}

		static bool IsInside(const std::vector<Edge> &edges, Vector2 point)
		{
			bool inside = false;
			for(const Edge &edge : edges)
			{
				if((edge.start.y > point.y) != (edge.end.y > point.y) && point.x < GetEdgeXAtY(edge, point.y))
				{
					inside = !inside;
				}
			}

			return inside;
		}

		static Status GetBoundaryEdges(const std::vector<Edge> &edges, double epsilon, std::vector<Edge> &result, bool offset = false, bool outer = false)
		{
			for(size_t i = 0; i < edges.size(); i++)
			{
				const Edge &edge = edges[i];
				if(offset && !edge.stroke)
				{
					continue;
				}

				const Vector2 direction = GetDifference(edge.end, edge.start);
				const double length = std::sqrt(direction.x * direction.x + direction.y * direction.y);
				if(length <= epsilon)
				{
					continue;
				}

				std::vector<double> splits = {0.0, 1.0};
				for(size_t j = 0; j < edges.size(); j++)
				{
					if(i == j)
					{
						continue;
					}
					double t;
					double u;
					if(GetEdgeIntersection(edge, edges[j], t, u))
					{
						splits.push_back(t);
					}
					// Include endpoints of collinear overlaps, which have no unique intersection.
					for(Vector2 p : {edges[j].start, edges[j].end})
					{
						const Vector2 delta = GetDifference(p, edge.start);
						if(std::abs(GetCrossProduct(direction, delta)) > epsilon * length)
						{
							continue;
						}
						t = (delta.x * direction.x + delta.y * direction.y) / (length * length);
						if(t > 0.0 && t < 1.0)
						{
							splits.push_back(t);
						}
					}
				}

				SortUniqueValues(splits, epsilon / length);
				for(size_t j = 1; j < splits.size(); j++)
				{
					const Vector2 middle = GetPointOnEdge(edge, (splits[j - 1] + splits[j]) * 0.5);
					const Vector2 normal = {-direction.y * epsilon * 4.0 / length, direction.x * epsilon * 4.0 / length};
					const Vector2 left = {middle.x + normal.x, middle.y + normal.y};
					const Vector2 right = {middle.x - normal.x, middle.y - normal.y};
					const bool leftInside = offset ? IsInsideOffset(edges, left, outer) : IsInside(edges, left);
					const bool rightInside = offset ? IsInsideOffset(edges, right, outer) : IsInside(edges, right);
					if(leftInside == rightInside)
					{
						continue;
					}

					const Edge boundary = {GetPointOnEdge(edge, splits[leftInside ? j - 1 : j]), GetPointOnEdge(edge, splits[leftInside ? j : j - 1]), false};
					bool duplicate = false;
					for(const Edge &other : result)
					{
						if((ArePointsEqual(boundary.start, other.start, epsilon) && ArePointsEqual(boundary.end, other.end, epsilon)) ||
						   (ArePointsEqual(boundary.start, other.end, epsilon) && ArePointsEqual(boundary.end, other.start, epsilon)))
						{
							duplicate = true;
							break;
						}
					}
					if(!duplicate)
					{
						if(result.size() >= MaximumEdges)
						{
							return Status::SubdivisionLimit;
						}
						result.push_back(boundary);
					}
				}
			}

			return Status::Success;
		}

		static Status AddStroke(const std::vector<Edge> &boundaries, double radius, double tolerance, double epsilon, std::vector<Edge> &edges)
		{
			Status status;
			const double pi = 3.14159265358979323846;
			std::vector<Vertex> vertices;
			for(size_t i = 0; i < boundaries.size(); i++)
			{
				const Edge &edge = boundaries[i];
				const Vector2 d = GetDifference(edge.end, edge.start);
				const double length = std::sqrt(d.x * d.x + d.y * d.y);
				const Vector2 n = {-d.y * radius / length, d.x * radius / length};
				if((status = AddPolygon({{edge.start.x + n.x, edge.start.y + n.y}, {edge.start.x - n.x, edge.start.y - n.y}, {edge.end.x - n.x, edge.end.y - n.y}, {edge.end.x + n.x, edge.end.y + n.y}}, true, edges)) != Status::Success)
				{
					return status;
				}
				for(Vector2 p : {edge.start, edge.end})
				{
					size_t vertex = 0;
					while(vertex < vertices.size() && !ArePointsEqual(vertices[vertex].point, p, epsilon))
					{
						vertex++;
					}
					if(vertex == vertices.size())
					{
						vertices.push_back({p, {}});
					}
					vertices[vertex].edges.push_back(i);
				}
			}

			const double step = 2.0 * std::acos(std::max(0.0, 1.0 - std::min(tolerance / radius, 1.0)));
			if(step <= 0.0)
			{
				return Status::SubdivisionLimit;
			}
			for(const Vertex &vertex : vertices)
			{
				double start = 0.0;
				double sweep = 2.0 * pi;
				if(vertex.edges.size() == 2)
				{
					Vector2 directions[2];
					for(size_t i = 0; i < 2; i++)
					{
						const Edge &edge = boundaries[vertex.edges[i]];
						Vector2 d = GetDifference(ArePointsEqual(edge.start, vertex.point, epsilon) ? edge.end : edge.start, vertex.point);
						const double length = std::sqrt(d.x * d.x + d.y * d.y);
						directions[i] = {d.x / length, d.y / length};
					}
					// Rectangles cover the inside of a bend; only add the outside sector.
					sweep = pi - std::acos(std::max(-1.0, std::min(1.0, directions[0].x * directions[1].x + directions[0].y * directions[1].y)));
					if(sweep <= 1.0e-7)
					{
						continue;
					}
					start = std::atan2(-directions[0].y - directions[1].y, -directions[0].x - directions[1].x) - sweep * 0.5;
				}

				const size_t count = static_cast<size_t>(std::ceil(sweep / step));
				if(count + edges.size() + 2 > MaximumEdges)
				{
					return Status::SubdivisionLimit;
				}

				std::vector<Vector2> sector;
				sector.push_back(vertex.point);
				for(size_t i = 0; i <= count; i++)
				{
					const double angle = start + sweep * i / count;
					sector.push_back({vertex.point.x + radius * std::cos(angle), vertex.point.y + radius * std::sin(angle)});
				}

				if((status = AddPolygon(sector, true, edges)) != Status::Success)
				{
					return status;
				}
			}
			return Status::Success;
		}

		static Status AddQuadraticSegmentsForCubic(const std::vector<Vector2> &p, double tolerance, unsigned depth, Path &path)
		{
			Status status;
			const Vector2 control = {(-p[0].x + 3.0 * p[1].x + 3.0 * p[2].x - p[3].x) * 0.25,
									 (-p[0].y + 3.0 * p[1].y + 3.0 * p[2].y - p[3].y) * 0.25};
			const Vector2 error1 = {(p[0].x + 2.0 * control.x) / 3.0 - p[1].x, (p[0].y + 2.0 * control.y) / 3.0 - p[1].y};
			const Vector2 error2 = {(p[3].x + 2.0 * control.x) / 3.0 - p[2].x, (p[3].y + 2.0 * control.y) / 3.0 - p[2].y};
			if(std::max(error1.x * error1.x + error1.y * error1.y, error2.x * error2.x + error2.y * error2.y) <= tolerance * tolerance)
			{
				PathSegment segment;
				segment.type = PathSegment::TypeBezierQuadratic;
				segment.controlPoints = {p.front(), control, p.back()};
				path.segments.push_back(segment);
				return Status::Success;
			}
			if(depth == 24)
			{
				return Status::SubdivisionLimit;
			}

			std::vector<Vector2> left;
			std::vector<Vector2> right;
			SubdivideControlPoints(p, left, right);
			if((status = AddQuadraticSegmentsForCubic(left, tolerance, depth + 1, path)) != Status::Success)
			{
				return status;
			}
			if((status = AddQuadraticSegmentsForCubic(right, tolerance, depth + 1, path)) != Status::Success)
			{
				return status;
			}
			return Status::Success;
		}

		static bool IsInsideOffset(const std::vector<Edge> &edges, Vector2 point, bool outer)
		{
			bool inside = false;
			int stroke = 0;
			for(const Edge &edge : edges)
			{
				if((edge.start.y > point.y) != (edge.end.y > point.y) && point.x < GetEdgeXAtY(edge, point.y))
				{
					if(edge.stroke)
					{
						stroke += edge.end.y > edge.start.y ? 1 : -1;
					}
					else
					{
						inside = !inside;
					}
				}
			}

			return outer ? (inside || stroke != 0) : (inside && stroke == 0);
		}

		static Vector2 EvaluateQuadratic(Vector2 a, Vector2 b, Vector2 c, double t)
		{
			const double s = 1.0 - t;
			return {s * s * a.x + 2.0 * s * t * b.x + t * t * c.x, s * s * a.y + 2.0 * s * t * b.y + t * t * c.y};
		}

		static void FitQuadratic(const std::vector<Vector2> &points, size_t first, size_t last, double tolerance, Path &path)
		{
			const Vector2 a = points[first];
			const Vector2 c = points[last];
			const Vector2 chord = GetDifference(c, a);
			const double chordLengthSquared = chord.x * chord.x + chord.y * chord.y;
			double lineErrorSquared = 0.0;
			for(size_t i = first + 1; i < last; i++)
			{
				const Vector2 delta = GetDifference(points[i], a);
				const double t = chordLengthSquared > 0.0 ? std::max(0.0, std::min(1.0, (delta.x * chord.x + delta.y * chord.y) / chordLengthSquared)) : 0.0;
				const Vector2 error = {delta.x - t * chord.x, delta.y - t * chord.y};
				lineErrorSquared = std::max(lineErrorSquared, error.x * error.x + error.y * error.y);
			}
			// Keep straight runs as lines to avoid unstable quadratic patches.
			if(last == first + 1 || lineErrorSquared <= tolerance * tolerance * 1.0e-12)
			{
				PathSegment segment;
				segment.type = PathSegment::TypeLine;
				segment.controlPoints = {a, c};
				path.segments.push_back(segment);
				return;
			}

			std::vector<double> parameters(last - first + 1, 0.0);
			for(size_t i = first + 1; i <= last; i++)
			{
				const Vector2 delta = GetDifference(points[i], points[i - 1]);
				parameters[i - first] = parameters[i - first - 1] + std::sqrt(delta.x * delta.x + delta.y * delta.y);
			}

			const double length = parameters.back();
			if(length == 0.0)
			{
				return;
			}
			for(double &parameter : parameters)
			{
				parameter /= length;
			}
			Vector2 b = {0.0, 0.0};
			double denominator = 0.0;
			for(size_t i = first + 1; i < last; i++)
			{
				const double t = parameters[i - first];
				const double s = 1.0 - t;
				const double weight = 2.0 * s * t;
				b.x += weight * (points[i].x - s * s * a.x - t * t * c.x);
				b.y += weight * (points[i].y - s * s * a.y - t * t * c.y);
				denominator += weight * weight;
			}
			b.x /= denominator;
			b.y /= denominator;
			double errorSquared = 0.0;
			size_t split = (first + last) / 2;
			for(size_t i = first; i < last; i++)
			{
				const double t0 = parameters[i - first];
				const double t1 = parameters[i + 1 - first];
				const Vector2 q0 = EvaluateQuadratic(a, b, c, t0);
				const Vector2 q1 = EvaluateQuadratic(a, b, c, t1);
				const Vector2 derivative = {2.0 * ((1.0 - t0) * (b.x - a.x) + t0 * (c.x - b.x)),
											2.0 * ((1.0 - t0) * (b.y - a.y) + t0 * (c.y - b.y))};
				const Vector2 middleControl = {q0.x + derivative.x * (t1 - t0) * 0.5, q0.y + derivative.y * (t1 - t0) * 0.5};
				const Vector2 lineMiddle = {(points[i].x + points[i + 1].x) * 0.5, (points[i].y + points[i + 1].y) * 0.5};
				// The difference curve's control points bound the error over this interval.
				for(Vector2 error : {GetDifference(q0, points[i]), GetDifference(middleControl, lineMiddle), GetDifference(q1, points[i + 1])})
				{
					const double squared = error.x * error.x + error.y * error.y;
					if(squared > errorSquared)
					{
						errorSquared = squared;
						split = std::max(first + 1, std::min(last - 1, i + 1));
					}
				}
			}
			if(errorSquared > tolerance * tolerance)
			{
				FitQuadratic(points, first, split, tolerance, path);
				FitQuadratic(points, split, last, tolerance, path);
				return;
			}
			PathSegment segment;
			segment.type = PathSegment::TypeBezierQuadratic;
			segment.controlPoints = {a, b, c};
			path.segments.push_back(segment);
		}

		static Status FitContours(const std::vector<Edge> &edges, double tolerance, double epsilon, PathCollection &result)
		{
			std::vector<bool> used(edges.size(), false);
			for(size_t start = 0; start < edges.size(); start++)
			{
				if(used[start])
				{
					continue;
				}

				std::vector<Vector2> points;
				size_t current = start;
				while(true)
				{
					used[current] = true;
					points.push_back(edges[current].start);
					if(ArePointsEqual(edges[current].end, edges[start].start, epsilon * 8.0))
					{
						break;
					}
					size_t next = edges.size();
					double bestTurn = -4.0;
					const Vector2 incoming = GetDifference(edges[current].end, edges[current].start);
					for(size_t i = 0; i < edges.size(); i++)
					{
						if(used[i] || !ArePointsEqual(edges[i].start, edges[current].end, epsilon * 8.0))
						{
							continue;
						}

						const Vector2 outgoing = GetDifference(edges[i].end, edges[i].start);
						const double turn = std::atan2(GetCrossProduct(incoming, outgoing), incoming.x * outgoing.x + incoming.y * outgoing.y);
						if(turn > bestTurn)
						{
							bestTurn = turn;
							next = i;
						}
					}
					if(next == edges.size())
					{
						return Status::TopologyFailure;
					}
					current = next;
				}
				if(points.size() < 3)
				{
					continue;
				}
				std::vector<size_t> corners;
				for(size_t i = 0; i < points.size(); i++)
				{
					const Vector2 a = GetDifference(points[i], points[(i + points.size() - 1) % points.size()]);
					const Vector2 b = GetDifference(points[(i + 1) % points.size()], points[i]);
					const double lengths = std::sqrt((a.x * a.x + a.y * a.y) * (b.x * b.x + b.y * b.y));
					if(lengths > 0.0 && (a.x * b.x + a.y * b.y) / lengths < 0.866025403784)
					{
						corners.push_back(i);
					}
				}
				if(corners.empty())
				{
					corners.push_back(0);
				}

				Path path;
				for(size_t i = 0; i < corners.size(); i++)
				{
					const size_t begin = corners[i];
					const size_t end = i + 1 < corners.size() ? corners[i + 1] : corners[0] + points.size();
					std::vector<Vector2> run;
					for(size_t j = begin; j <= end; j++)
					{
						run.push_back(points[j % points.size()]);
					}

					FitQuadratic(run, 0, run.size() - 1, tolerance, path);
				}
				if(!path.segments.empty())
				{
					result.paths.push_back(path);
				}
			}

			return Status::Success;
		}

		static PathCollection SplitExtrema(const PathCollection &paths)
		{
			PathCollection result;
			for(const Path &path : paths.paths)
			{
				Path split;
				for(const PathSegment &segment : path.segments)
				{
					if(segment.type != PathSegment::TypeBezierQuadratic)
					{
						split.segments.push_back(segment);
						continue;
					}

					const Vector2 a = segment.controlPoints[0];
					const Vector2 b = segment.controlPoints[1];
					const Vector2 c = segment.controlPoints[2];
					std::vector<double> parameters = {0.0, 1.0};
					const double numerator[2] = {a.x - b.x, a.y - b.y};
					const double denominator[2] = {a.x - 2.0 * b.x + c.x, a.y - 2.0 * b.y + c.y};
					for(size_t axis = 0; axis < 2; axis++)
					{
						if(denominator[axis] == 0.0)
						{
							continue;
						}

						const double t = numerator[axis] / denominator[axis];
						if(t > 0.0 && t < 1.0)
						{
							parameters.push_back(t);
						}
					}

					SortUniqueValues(parameters, std::numeric_limits<double>::epsilon() * 16.0);
					Vector2 start = a;
					for(size_t i = 1; i < parameters.size(); i++)
					{
						const double t = parameters[i - 1];
						const double endT = parameters[i];
						const Vector2 end = i + 1 == parameters.size() ? c : EvaluateQuadratic(a, b, c, endT);
						const Vector2 control = {start.x + (endT - t) * ((1.0 - t) * (b.x - a.x) + t * (c.x - b.x)),
												 start.y + (endT - t) * ((1.0 - t) * (b.y - a.y) + t * (c.y - b.y))};
						PathSegment part;
						part.type = PathSegment::TypeBezierQuadratic;
						part.controlPoints = {start, control, end};
						split.segments.push_back(part);
						start = end;
					}
				}
				result.paths.push_back(split);
			}

			return result;
		}

		static void AppendMesh(const TriangleMesh &source, float role, TriangleMesh &target)
		{
			const uint32_t base = static_cast<uint32_t>(target.vertices.size() / 6);
			for(size_t i = 0; i < source.vertices.size(); i += 5)
			{
				target.vertices.insert(target.vertices.end(), source.vertices.begin() + i, source.vertices.begin() + i + 5);
				target.vertices.push_back(role);
			}
			for(uint32_t index : source.indices)
			{
				target.indices.push_back(base + index);
			}
		}

		static void AppendPatch(const PathSegment &segment, float direction, float role, TriangleMesh &mesh)
		{
			const float uv[3][2] = {{0.0f, 0.0f}, {0.5f, 0.0f}, {1.0f, 1.0f}};
			for(size_t i = 0; i < 3; i++)
			{
				mesh.indices.push_back(static_cast<uint32_t>(mesh.vertices.size() / 6));
				mesh.vertices.insert(mesh.vertices.end(), {static_cast<float>(segment.controlPoints[i].x), static_cast<float>(segment.controlPoints[i].y), uv[i][0], uv[i][1], direction, role});
			}
		}

		static void JoinTouchingContours(TriangulatorBruteForce::Polygon &polygon, double epsilon)
		{
			// Remove duplicate edges where fill meets the silhouette.
			for(TriangulatorBruteForce::Outline &outline : polygon.outlines)
			{
				if(outline.points.size() > 1 && ArePointsEqual(outline.points.front(), outline.points.back(), epsilon))
				{
					outline.points.pop_back();
				}
			}
			for(size_t a = 0; a < polygon.outlines.size(); a++)
			{
				for(size_t b = a + 1; b < polygon.outlines.size(); b++)
				{
					std::vector<Vector2> &first = polygon.outlines[a].points;
					const std::vector<Vector2> &second = polygon.outlines[b].points;
					bool joined = false;
					for(size_t i = 0; i < first.size() && !joined; i++)
					{
						for(size_t j = 0; j < second.size(); j++)
						{
							const bool same = ArePointsEqual(first[i], second[j], epsilon) && ArePointsEqual(first[(i + 1) % first.size()], second[(j + 1) % second.size()], epsilon);
							const bool reversed = ArePointsEqual(first[i], second[(j + 1) % second.size()], epsilon) && ArePointsEqual(first[(i + 1) % first.size()], second[j], epsilon);
							if(!same && !reversed)
							{
								continue;
							}

							std::vector<Vector2> points;
							for(size_t k = 1; k <= first.size(); k++)
							{
								points.push_back(first[(i + k) % first.size()]);
							}
							for(size_t k = 1; k + 1 < second.size(); k++)
							{
								points.push_back(second[same ? (j + second.size() - k) % second.size() : (j + 1 + k) % second.size()]);
							}
							first = points;
							polygon.outlines.erase(polygon.outlines.begin() + b);
							b--;
							joined = true;
							break;
						}
					}
				}
			}
		}

		static Status GenerateAnalyticMesh(const PathCollection &silhouette, const PathCollection &fill, double epsilon, TriangleMesh &result)
		{
			PathCollection combined = silhouette;
			combined.paths.insert(combined.paths.end(), fill.paths.begin(), fill.paths.end());
			combined = MeshGeneratorLoopBlinn::FilterDegenerateSegments(combined, epsilon * epsilon);
			// The winding classifier needs extrema included in the endpoint bounds.
			combined = SplitExtrema(combined);
			combined = MeshGeneratorLoopBlinn::ResolveIntersections(combined);
			// Shared patches must be disjoint, even at small coordinate scales.
			size_t remainingSplits = MaximumEdges;
			for(const Path &path : combined.paths)
			{
				if(path.segments.size() > remainingSplits)
				{
					return Status::SubdivisionLimit;
				}
				remainingSplits -= path.segments.size();
			}
			PathCollection resolved;
			if(!MeshGeneratorLoopBlinn::ResolveOverlaps(combined, 0.0, &remainingSplits, resolved))
			{
				return Status::SubdivisionLimit;
			}
			combined = std::move(resolved);
			combined = MeshGeneratorLoopBlinn::FindWindingOrder(combined);
			if(combined.paths.size() != silhouette.paths.size() + fill.paths.size())
			{
				return Status::TopologyFailure;
			}

			TriangleMesh mesh;
			mesh.features = {TriangleMesh::VertexFeaturePosition, TriangleMesh::VertexFeatureUV, TriangleMesh::VertexFeatureOutline};
			TriangulatorBruteForce::Polygon ringPolygon, fillPolygon;
			for(size_t i = 0; i < combined.paths.size(); i++)
			{
				const bool internal = i >= silhouette.paths.size();
				TriangulatorBruteForce::Outline ringContour, fillContour;
				for(const PathSegment &segment : combined.paths[i].segments)
				{
					if(ringContour.points.empty())
					{
						ringContour.points.push_back(segment.controlPoints.front());
					}
					if(internal && fillContour.points.empty())
					{
						fillContour.points.push_back(segment.controlPoints.front());
					}
					if(segment.type == PathSegment::TypeBezierQuadratic)
					{
						if(!segment.isFilledOutside)
						{
							ringContour.points.push_back(segment.controlPoints[1]);
						}
						if(internal && segment.isFilledOutside)
						{
							fillContour.points.push_back(segment.controlPoints[1]);
						}

						const float ringDirection = segment.isFilledOutside ? 1.0f : -1.0f;
						// +/-2 marks a shared patch that blends fill and outline once.
						if(internal)
						{
							AppendPatch(segment, -2.0f * ringDirection, 0.0f, mesh);
						}
						else
						{
							AppendPatch(segment, ringDirection, 1.0f, mesh);
						}
					}
					ringContour.points.push_back(segment.controlPoints.back());
					if(internal)
					{
						fillContour.points.push_back(segment.controlPoints.back());
					}
				}
				ringPolygon.outlines.push_back(ringContour);
				if(internal)
				{
					fillPolygon.outlines.push_back(fillContour);
				}
			}

			JoinTouchingContours(ringPolygon, epsilon);
			if(!ringPolygon.outlines.empty())
			{
				AppendMesh(TriangulatorBruteForce::Triangulate(ringPolygon), 1.0f, mesh);
			}
			if(!fillPolygon.outlines.empty())
			{
				AppendMesh(TriangulatorBruteForce::Triangulate(fillPolygon), 0.0f, mesh);
			}

			result = std::move(mesh);
			return Status::Success;
		}
	};

	MeshGeneratorLoopBlinn::Status MeshGeneratorLoopBlinn::GetMeshForContours(const PathCollection &silhouette, const PathCollection &fill, TriangleMesh &mesh)
	{
		mesh = TriangleMesh();
		return LoopBlinnOutlineBuilder::Build(silhouette, fill, mesh);
	}

	MeshGeneratorLoopBlinn::Status MeshGeneratorLoopBlinn::GetMeshForPathCollection(const PathCollection &paths, double width, TriangleMesh &mesh)
	{
		mesh = TriangleMesh();
		return LoopBlinnOutlineBuilder::Build(paths, width, mesh);
	}
} // namespace KG
