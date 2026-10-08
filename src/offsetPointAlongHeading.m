function pointOut = offsetPointAlongHeading(pointIn, heading, distance)
    pointOut = pointIn;
    pointOut(1) = pointIn(1) + distance * sin(heading);
    pointOut(2) = pointIn(2) + distance * cos(heading);
end
