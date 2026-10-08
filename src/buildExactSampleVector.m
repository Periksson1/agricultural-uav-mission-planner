function vec = buildExactSampleVector(startVal, endVal, ds)
    if abs(endVal - startVal) <= 1e-12
        vec = startVal;
        return
    end

    sgn = sign(endVal - startVal);
    nFull = floor(abs(endVal - startVal) / ds);
    vec = startVal + sgn * (0:nFull) * ds;
    if abs(vec(end) - endVal) > 1e-9
        vec = [vec endVal];
    end
end
