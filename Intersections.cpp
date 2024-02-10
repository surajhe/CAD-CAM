    bool IntersectionLineLine(const math::simple_segment_t& seg1, const math::simple_segment_t& seg2)
    {
      math::vector3d_t origin_1    = std::get<0>(seg1);
      math::vector3d_t direction_1 = std::get<1>(seg1);
      math::vector3d_t origin_2    = std::get<0>(seg2);
      math::vector3d_t direction_2 = std::get<1>(seg2);

      math::vector3d_t n = math::CrossProduct(direction_1, direction_2);
      double magnitude = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
      
      // Check if the lines are parallel (denominator close to 0)
      if (std::abs(magnitude) < std::numeric_limits<double>::epsilon())
        return false;

      // Calculate vector between origins
      math::vector3d_t p = { origin_2[0] - origin_1[0], origin_2[1] - origin_1[1], origin_2[2] - origin_1[2] };

      // Calculate parameters t and u
      double t = math::DotProduct(math::CrossProduct(p, direction_2), n) / magnitude;
      double u = math::DotProduct(math::CrossProduct(p, direction_1), n) / magnitude;

      // Check if intersection point lies within both segments' intervals
      math::interval_t interval1 = std::get<4>(seg1);
      math::interval_t interval2 = std::get<4>(seg2);

      if ((t > interval1.first && t < interval1.second) && (u > interval2.first && u < interval2.second))
        return true; // Segments intersect

      return false; // Segments do not intersect
    }

    // Function to check if a point lies within an angular interval
    bool IsInInterval(double angle, const math::interval_t& interval) {
      double start = interval.first;
      double end = interval.second;
      if (start <= end) {
        return (angle >= start && angle <= end);
      }
      else {
        return (angle >= start || angle <= end);
      }
    }

    bool IntersectionLineCircle(const math::simple_segment_t& line, const math::simple_segment_t& arc)
    {
      math::vector3d_t line_origin = std::get<0>(line); 
      math::vector3d_t line_direction = std::get<1>(line);
      math::vector3d_t arc_origin = std::get<0>(arc);
      math::vector3d_t arc_axis = std::get<2>(arc);
      double arc_radius = std::get<3>(arc);
      math::interval_t arc_interval = std::get<4>(arc);

      // Calculate cross product of line direction and arc axis
      math::vector3d_t cross_dir_axis = math::CrossProduct(line_direction, arc_axis);

      // Calculate dot product of cross product and arc axis
      double dot_cross_dir_axis_axis = math::DotProduct(cross_dir_axis, arc_axis);

      // Check if line is parallel or nearly parallel to arc axis
      if (std::abs(dot_cross_dir_axis_axis) < std::numeric_limits<double>::epsilon())
        return false;

      // Calculate vector from arc origin to line origin
      math::vector3d_t p = { line_origin[0] - arc_origin[0], line_origin[1] - arc_origin[1], line_origin[2] - arc_origin[2] };

      // Calculate parameter t for line equation
      double t = math::DotProduct(math::CrossProduct(p, arc_axis), arc_axis) / dot_cross_dir_axis_axis;

      // Calculate intersection point on line
      math::vector3d_t intersection_point = { line_origin[0] + t * line_direction[0],
                                       line_origin[1] + t * line_direction[1],
                                       line_origin[2] + t * line_direction[2] };

      // Calculate vector from arc origin to intersection point
      math::vector3d_t q = { intersection_point[0] - arc_origin[0], intersection_point[1] - arc_origin[1], intersection_point[2] - arc_origin[2] };

      // Calculate angle between arc axis and q
      double angle = std::atan2(math::DotProduct(math::CrossProduct(q, arc_axis), arc_axis), math::DotProduct(q, arc_axis));

      // Check if intersection point lies within arc interval
      return (IsInInterval(angle, arc_interval));

    }

    bool IntersectionCircleCircle(const math::simple_segment_t& arc1, const math::simple_segment_t& arc2)
    {
      // Extract properties of arc1
      math::vector3d_t origin1 = std::get<0>(arc1);
      math::vector3d_t direction1 = std::get<1>(arc1);
      math::vector3d_t axis1 = std::get<2>(arc1);
      double radius1 = std::get<3>(arc1);
      math::interval_t interval1 = std::get<4>(arc1);

      // Extract properties of arc2
      math::vector3d_t origin2 = std::get<0>(arc2);
      math::vector3d_t direction2 = std::get<1>(arc2);
      math::vector3d_t axis2 = std::get<2>(arc2);
      double radius2 = std::get<3>(arc2);
      math::interval_t interval2 = std::get<4>(arc2);

      // Calculate vector between origins
      math::vector3d_t delta = { origin2[0] - origin1[0], origin2[1] - origin1[1], origin2[2] - origin1[2] };
      double distance_squared = math::DotProduct(delta, delta);

      // Calculate distance between origins
      double distance = std::sqrt(distance_squared);

      // Check if arcs/circles are separate or one contains the other
      if (distance > radius1 + radius2 || distance < std::abs(radius1 - radius2))    // No Intersection
        return false;

      // Calculate intersection angles
      double angle1 = std::atan2(delta[1], delta[0]);
      double angle2 = angle1 + std::acos((distance_squared + radius1 * radius1 - radius2 * radius2) / (2 * distance * radius1));

      // Check if angles are within intervals
      if ((IsInInterval(angle1, interval1) && IsInInterval(angle1, interval2)) || (IsInInterval(angle2, interval1) && IsInInterval(angle2, interval2)))
        return true;

      return false;
    }

    bool Intersections(const math::simple_segment_t& segment1, const math::simple_segment_t& segment2)
    {
      if (math::IsLine(segment1))
      {
        if (math::IsLine(segment2))
        {
          return IntersectionLineLine(segment1, segment2);
        }
        else
        {
          return IntersectionLineCircle(segment1, segment2);
        }
      }
      else
      {
        if (math::IsLine(segment2))
        {
          return IntersectionLineCircle(segment2, segment1);
        }
        else
        {
          return IntersectionCircleCircle(segment1, segment2);
        }
      }
      return false;
    }
}
