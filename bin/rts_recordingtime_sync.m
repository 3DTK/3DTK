% script for interpolating data of RTS Hannover

function output = recordingtime_sync(input, ref)

[input_m, input_n] = size(input);
[ref_m, ref_n] = size(ref);

output = zeros(ref_m, input_n);

for i = 1 : ref_m
    upper_index = find(input(:,1) > ref(i,1));
    
    upper = input(upper_index(1),:);

    if(upper_index(1) > 1)
        lower = input((upper_index(1)-1),:);
    else
        lower = upper;
    end
    
    diff = upper - lower;

    % normalise angel sym 0
    if(diff(5) > pi)
        diff(5) = diff(5) - 2*pi;
    end
    if(diff(5) < -pi)
        diff(5) = diff(5) + 2*pi;
    end
    
    if(diff(6) > pi)
        diff(6) = diff(6) - 2*pi;
    end
    if(diff(6) < -pi)
        diff(6) = diff(6) + 2*pi;
    end
    
    if(diff(7) > pi)
        diff(7) = diff(7) - 2*pi;
    end
    if(diff(7) < -pi)
        diff(7) = diff(7) + 2*pi;
    end
    
    if(diff(1) == 0)
        rel_time = 0;
    else
        rel_time = (ref(i,1) - lower(1,1)) / diff(1);
    end

    output(i,:) = lower + rel_time .* diff;
    
    % normalise angel sym 0
    if(output(i,5) > pi)
        output(i,5) = output(i,5) - 2*pi;
    end
    if(output(i,5) < -pi)
        output(i,5) = output(i,5) + 2*pi;
    end
    
    if(output(i,6) > pi)
        output(i,6) = output(i,6) - 2*pi;
    end
    if(output(i,6) < -pi)
        output(i,6) = output(i,6) + 2*pi;
    end
    
    if(output(i,7) > (2*pi))
        output(i,7) = output(i,7) - 2*pi;
    end
    if(output(i,7) < 0.0)
        output(i,7) = output(i,7) + 2*pi;
    end
    
end
