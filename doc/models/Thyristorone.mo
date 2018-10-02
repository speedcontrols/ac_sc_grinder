class Thyristorone
  ////automatically generated ////
      //parameters
      parameter Real Rsnubber = 1.000000e+03;
      parameter Real Rz = 1.000000e+10;
      //input variables
      Real u1;
      Real Isnubber;
      Real active(start=0.0);
      Pin anode;
      //output variables
      Pin cathode;
  ////do not modif above this line ////

 //     Real x(start=1), y(start=2);
equation
      Isnubber = (anode.v-cathode.v)/Rsnubber;
      active = if (Isnubber>0.0) and ((u1>0.5)or(active>0.0)) then 1.0 else 0.0;
      anode.i = if active<1.0 then (anode.v-cathode.v)/Rz+Isnubber else (anode.v-cathode.v)/0.01+Isnubber;
      cathode.i = -anode.i;

end Thyristorone;
