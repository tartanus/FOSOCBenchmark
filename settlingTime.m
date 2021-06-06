%% settling time measure function
%y: system output
%w: sliding window size
function ts=settlingTime(y,w,time)

    for i=1:length(y)
        
        if std(y(i:i+w))<1e-3
            ts=time(i);
            break
        else
            ts=time(end);
        end
        
        
    end




end

