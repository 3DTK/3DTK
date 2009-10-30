function output = recordingtime_gps(input, ref)

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

    if(diff(1) == 0)
        rel_time = 0;
    else
        rel_time = (ref(i,1) - lower(1,1)) / diff(1);
    end

    output(i,:) = lower + rel_time .* diff;
    
end
