function [smoothHeadingDegrees] = Fixed_Heading(mag,mag_bias,declinationAngle,k)
    mag(:,k) = mag(:,k) - mag_bias'; % magnetic bias error
    heading(k) = atan2(mag(2,k), mag(1,k)); % calculate heading angle (rad)
    heading(k) = heading(k) + declinationAngle;
    %   Correct for heading < 0deg and heading > 360deg
    if heading(k) < 0
        heading(k) = heading(k) + 2*pi;
    end
    if heading(k) > 2*pi
        heading(k) = heading(k) - 2*pi;
    end
    
    fixedHeadingDegrees(k) = heading(k)*180/pi; % rad to deg
    % Smooth angles rotation for +/- 3deg
    smoothHeadingDegrees(k) = round( fixedHeadingDegrees(k) );
    if k > 2
        if (smoothHeadingDegrees(k) < (smoothHeadingDegrees(k-1) + 3) && smoothHeadingDegrees(k) > (smoothHeadingDegrees(k-1) - 3))
            smoothHeadingDegrees(k) = smoothHeadingDegrees(k-1);
        end
    end
end