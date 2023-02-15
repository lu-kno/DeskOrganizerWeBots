% REMOVE NOISE: Peak Identification

% The Linear CleanUp is done in three steps with increasing time tolerance
for n=9:-4:1   
	In_Peak = 0;
	stable = 0;
	Time_Tol = Filt_Time/n;
	for i=2:length(SIGNAL)-1
		if ~In_Peak && abs(SIGNAL(i)-SIGNAL(i-1)) >= Tol_Up(i,1)/2
			Peak_Entry_diff=SIGNAL(i)-SIGNAL(i-1);
			Peak_Entry_ind=i;
			In_Peak=1;

		elseif In_Peak && Time(i)-Time(Peak_Entry_ind) > Time_Tol/1000 
			In_Peak = 0;		% To many points to be taken as a peak
	
		elseif In_Peak && abs(SIGNAL(i+1)-SIGNAL(i)) < Tol_Up(i,1)/2
			if (abs(SIGNAL(i)-SIGNAL(Peak_Entry_ind-1)) ...
			       < abs(Peak_Entry_diff)*0.2 ...
			     ||  abs(SIGNAL(i)-SIGNAL(1+(abs(Peak_Entry_ind-4) ...
			      	                       +(Peak_Entry_ind-4))/2)) ...
		    	   < abs(Peak_Entry_diff)*0.2)

				stable=stable+1;
				if stable == 2
					Diff = SIGNAL(i)-SIGNAL(Peak_Entry_ind-1);
					for p=0:(i-Peak_Entry_ind)
					  SIGNAL(Peak_Entry_ind+p) = SIGNAL(Peak_Entry_ind-1) ...
					                           + Diff*(p+1)/(i-Peak_Entry_ind+1);
					end
					In_Peak = 0;
					stable = 0;
				end
			end
		end
	end
end