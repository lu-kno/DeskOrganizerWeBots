%REMOVE NOISE: Simple Moving Average

raster_ord = round(log10((Time(200)-Time(100))/100))+3;
raster = 10^raster_ord;
L = Filt_Time/raster;
Tmp = SIGNAL;

for i=1:length(SIGNAL)
	trailArr(mod(i-1,L)+1) = Tmp(i);
	SIGNAL(i) = mean(trailArr);
end