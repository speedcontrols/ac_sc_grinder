scs_m = xcosDiagramToScilab('model_grinder.zcos');

speed = 3000; // Real motor speed in RPM

// Model parameters

scs_m.props.context = [..
"Speed = "+string(speed).. 
"angle = 0.15".. // Triac firing angle, 0 - 1
"R_motor = 92".. // Motor resistance
"L_motor = 0.17".. // Motor inductance
"coeff_motor = 0.07".. // (Back-EMF / (Speed * Current)) coefficient
"current_quant = 0.001611328125".. // 1 bit of ADC value in amperes
"voltage_quant = 0.161938476562".. // 1 bit of ADC value in volts
"start_measure = 0.01925".. // Speed measuring starts at this time
"tick_freq = 17857".. // Sampling time of speed calculator
"stop_time = 0.02".. // Speed measuring stops at this time
];

xcos_simulate(scs_m, 4);

scf();

x = [0:0.1:0.9]';
xset("thickness",3);
plot2d(x,[(ones(1,10)*speed)' (ones(1,10)*m_speed.values)'],rect=[0,0,1,30000]);

xset("thickness",0);
xgrid();
title(['Results of formula' '$\LARGE{n = \frac{Voltage}{Current} - R - L \frac{dCurrent}{dt} \frac{1}{Current}}$']);
legend(['real' 'measured'],1);

printf("\n Calculated Speed = %f", m_speed.values);

