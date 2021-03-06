within ;
package Thesis_Bram_Stockman

  model Pump
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan(
      redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater,
      redeclare IDEAS.Fluid.Movers.Data.Generic per,
      nominalValuesDefineDefaultPressureCurve=true,
      riseTime=30,
      m_flow_start=0.1,
      use_inputFilter=true,
      init=Modelica.Blocks.Types.Init.SteadyState,
      inputType=IDEAS.Fluid.Types.InputType.Continuous,
      m_flow_nominal=10)
      annotation (Placement(transformation(extent={{-42,18},{-22,38}})));
    IDEAS.Fluid.FixedResistances.HydraulicDiameter res(
      length=100,
      redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater,
      m_flow_nominal=1)
      annotation (Placement(transformation(extent={{-40,-36},{-20,-16}})));
    IDEAS.Fluid.Sources.Boundary_pT bou(
      nPorts=2,
      redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater,
      p=100000) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={26,18})));
    Modelica.Blocks.Sources.Step step(height=10, startTime=5)
      annotation (Placement(transformation(extent={{-100,58},{-80,78}})));
  equation
    connect(res.port_a, fan.port_a) annotation (Line(points={{-40,-26},{-54,-26},
            {-54,28},{-42,28}}, color={0,127,255}));
    connect(bou.ports[1], res.port_b) annotation (Line(points={{24,28},{66,28},
            {66,-26},{-20,-26}}, color={0,127,255}));
    connect(fan.port_b, bou.ports[2]) annotation (Line(points={{-22,28},{28,28}},
                               color={0,127,255}));
    connect(step.y, fan.m_flow_in) annotation (Line(points={{-79,68},{-56,68},{
            -56,40},{-32,40}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Pump;

  model HeatPumpSystem
    package Medium = IDEAS.Media.Water;
    IDEAS.Fluid.HeatPumps.HP_WaterWater_OnOff hP_WaterWater_OnOff(
      m1_flow_nominal=1,
      m2_flow_nominal=1,
      redeclare
        IDEAS.Fluid.HeatPumps.Data.PerformanceMaps.VitoCal300GBWS301dotA08
        heatPumpData,
      redeclare package Medium1 = Medium,
      redeclare package Medium2 = Medium,
      use_onOffSignal=true)
      annotation (Placement(transformation(extent={{10,-10},{-10,10}},
          rotation=90,
          origin={-20,70})));
    IDEAS.Fluid.Geothermal.Borefields.OneUTube borFie(
        redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater, borFieDat=
          IDEAS.Fluid.Geothermal.Borefields.Data.Borefield.Example(conDat=
          IDEAS.Fluid.Geothermal.Borefields.Data.Configuration.Example(
            borCon=IDEAS.Fluid.Geothermal.Borefields.Types.BoreholeConfiguration.SingleUTube,
            use_Rb=true,
            Rb=0.26,
            mBor_flow_nominal=0.17,
            mBorFie_flow_nominal=mBor_flow_nominal*nBor,
            rBor=0.0625,
            dBor=1,
            cooBor={{0,0},{0,6},{6,0},{6,6}},
            rTub=0.0125,
            kTub=0.43,
            eTub=0.002,
            xC=0.03)))
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-74,6})));
    IDEAS.Fluid.MixingVolumes.MixingVolume vol(
      m_flow_nominal=1,
      redeclare package Medium = Medium,
      V=100,
      nPorts=4,
      T_start=308.15)                                       annotation (
        Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=90,
          origin={40,70})));
                IDEAS.Templates.Structure.Case900FloorHeating
                                                  structure(T_start=294.15)
                                                       "Building structure"
      annotation (Placement(transformation(extent={{-44,-82},{-10,-60}})),
        __Dymola_choicesAllMatching=true);
    IDEAS.Fluid.Sources.Boundary_pT bou(
      redeclare package Medium = Medium,
      nPorts=1,
      p=200000)
      annotation (Placement(transformation(extent={{-122,70},{-102,90}})));
    IDEAS.Fluid.Sources.Boundary_pT bou1(
      nPorts=1,
      redeclare package Medium = Medium,
      p=200000)
      annotation (Placement(transformation(extent={{-28,26},{-8,46}})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan(
      m_flow_nominal=1,
      redeclare IDEAS.Fluid.Movers.Data.Generic per,
      redeclare package Medium = Medium,
      inputType=IDEAS.Fluid.Types.InputType.Constant,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      addPowerToMedium=false,
      tau=60,
      use_inputFilter=false)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-88,38})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan1(
      m_flow_nominal=1,
      redeclare IDEAS.Fluid.Movers.Data.Generic per,
      redeclare package Medium = Medium,
      inputType=IDEAS.Fluid.Types.InputType.Constant)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=0,
          origin={12,80})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan2(
      redeclare IDEAS.Fluid.Movers.Data.Generic per,
      redeclare package Medium = Medium,
      inputType=IDEAS.Fluid.Types.InputType.Constant,
      m_flow_nominal=5)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=270,
          origin={74,36})));
    IDEAS.Fluid.HeatExchangers.RadiantSlab.EmbeddedPipe embeddedPipe(
      redeclare package Medium = Medium,
      m_flow_nominal=1,
      A_floor=100)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=180,
          origin={58,-38})));
    Modelica.Blocks.Logical.Hysteresis hysteresis(uLow=33 + 273, uHigh=37 + 273)
      annotation (Placement(transformation(extent={{16,144},{-4,124}})));
    Modelica.Blocks.Logical.Not not1 annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-22,134})));
    Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
      annotation (Placement(transformation(extent={{54,102},{74,122}})));
  equation
    connect(fan.port_a, borFie.port_b)
      annotation (Line(points={{-88,28},{-88,6},{-84,6}}, color={0,127,255}));
    connect(fan.port_b, hP_WaterWater_OnOff.port_a1) annotation (Line(points={{
            -88,48},{-88,80},{-26,80}}, color={0,127,255}));
    connect(bou.ports[1], hP_WaterWater_OnOff.port_a1)
      annotation (Line(points={{-102,80},{-26,80}}, color={0,127,255}));
    connect(hP_WaterWater_OnOff.port_b2, fan1.port_a)
      annotation (Line(points={{-14,80},{2,80}}, color={0,127,255}));
    connect(hP_WaterWater_OnOff.port_a2, vol.ports[1]) annotation (Line(points=
            {{-14,60},{24,60},{24,67},{30,67}}, color={0,127,255}));
    connect(fan1.port_b, vol.ports[2]) annotation (Line(points={{22,80},{24,80},
            {24,69},{30,69}}, color={0,127,255}));
    connect(bou1.ports[1], hP_WaterWater_OnOff.port_a2) annotation (Line(points=
           {{-8,36},{-4,36},{-4,60},{-14,60}}, color={0,127,255}));
    connect(hP_WaterWater_OnOff.port_b1, borFie.port_a) annotation (Line(points=
           {{-26,60},{-56,60},{-56,6},{-64,6}}, color={0,127,255}));
    connect(fan2.port_a, vol.ports[3]) annotation (Line(points={{74,46},{72,46},
            {72,92},{30,92},{30,71}}, color={0,127,255}));
    connect(fan2.port_b, embeddedPipe.port_a)
      annotation (Line(points={{74,26},{74,-38},{68,-38}}, color={0,127,255}));
    connect(embeddedPipe.port_b, vol.ports[4])
      annotation (Line(points={{48,-38},{30,-38},{30,73}}, color={0,127,255}));
    connect(embeddedPipe.heatPortEmb[1], structure.heatPortEmb[1]) annotation (
        Line(points={{58,-48},{58,-64.4},{-10,-64.4}}, color={191,0,0}));
    connect(hysteresis.y, not1.u)
      annotation (Line(points={{-5,134},{-10,134}}, color={255,0,255}));
    connect(not1.y, hP_WaterWater_OnOff.on) annotation (Line(points={{-33,134},
            {-36,134},{-36,72},{-30.8,72}}, color={255,0,255}));
    connect(temperatureSensor.port, vol.heatPort) annotation (Line(points={{54,
            112},{56,112},{56,60},{40,60}}, color={191,0,0}));
    connect(temperatureSensor.T, hysteresis.u) annotation (Line(points={{74,112},
            {82,112},{82,136},{18,136},{18,134}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-140,
              -100},{100,120}})),                                  Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-140,-100},{100,
              120}})),
      experiment(
        StopTime=604800,
        Tolerance=1e-06,
        __Dymola_fixedstepsize=10,
        __Dymola_Algorithm="Euler"),
      __Dymola_experimentSetupOutput,
      __Dymola_experimentFlags(
        Advanced(GenerateVariableDependencies=false, OutputModelicaCode=false),
        Evaluate=false,
        OutputCPUtime=false,
        OutputFlatModelica=false));
  end HeatPumpSystem;

  record DYNACIAT_200_LG_LGP_cissimmo_wetter =
    IDEAS.Fluid.HeatPumps.Data.ScrollWaterToWater.Generic (
      volRat = 2.30724270104,
      V_flow_nominal = 0.0114761424136,
      leaCoe = 0.0035247058965,
      etaEle = 0.917967648379,
      PLos = 198.349670073,
      dTSup = 0.00527806112236,
      UACon = 76516.2456947,
      UAEva = 138624.163733)
    annotation (
      defaultComponentPrefixes = "parameter",
      defaultComponentName="datHeaPum",
      preferredView="info",
    Documentation(info="<html>
<p>
Generated by Iago Cupeiro on 2017-12-06.
</p>
</html>"));
  model Simplified_heating_system
   package Medium = IDEAS.Media.Water;
    IDEAS.Fluid.HeatPumps.HP_WaterWater_OnOff hP_WaterWater_OnOff(
      m1_flow_nominal=1,
      m2_flow_nominal=1,
      redeclare
        IDEAS.Fluid.HeatPumps.Data.PerformanceMaps.VitoCal300GBWS301dotA08
        heatPumpData,
      redeclare package Medium1 = Medium,
      redeclare package Medium2 = Medium,
      use_onOffSignal=true)
      annotation (Placement(transformation(extent={{10,-10},{-10,10}},
          rotation=90,
          origin={-24,16})));
    IDEAS.Fluid.Geothermal.Borefields.OneUTube borFie(borFieDat=
          IDEAS.Fluid.Geothermal.Borefields.Data.Borefield.Example(),
        redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-78,-48})));
    IDEAS.Fluid.MixingVolumes.MixingVolume vol(
      m_flow_nominal=1,
      redeclare package Medium = Medium,
      nPorts=2,
      V=10,
      T_start=308.15)                                       annotation (
        Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=90,
          origin={36,16})));
    IDEAS.Fluid.Sources.Boundary_pT bou(
      redeclare package Medium = Medium,
      nPorts=1,
      p=200000)
      annotation (Placement(transformation(extent={{-126,16},{-106,36}})));
    IDEAS.Fluid.Sources.Boundary_pT bou1(
      nPorts=1,
      redeclare package Medium = Medium,
      p=200000)
      annotation (Placement(transformation(extent={{-32,-28},{-12,-8}})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan(
      m_flow_nominal=1,
      redeclare IDEAS.Fluid.Movers.Data.Generic per,
      redeclare package Medium = Medium,
      inputType=IDEAS.Fluid.Types.InputType.Continuous)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-92,-16})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan1(
      m_flow_nominal=1,
      redeclare IDEAS.Fluid.Movers.Data.Generic per,
      redeclare package Medium = Medium,
      inputType=IDEAS.Fluid.Types.InputType.Continuous)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=0,
          origin={8,26})));
    Modelica.Blocks.Logical.Hysteresis hysteresis(uLow=34 + 273, uHigh=36 + 273)
      annotation (Placement(transformation(extent={{12,90},{-8,70}})));
    Modelica.Blocks.Logical.Not not1 annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-26,80})));
    Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
      annotation (Placement(transformation(extent={{50,48},{70,68}})));
    Modelica.Blocks.Math.BooleanToReal booleanToReal annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-58,60})));
  equation
    connect(fan.port_a, borFie.port_b) annotation (Line(points={{-92,-26},{-92,
            -48},{-88,-48}}, color={0,127,255}));
    connect(fan.port_b, hP_WaterWater_OnOff.port_a1) annotation (Line(points={{
            -92,-6},{-92,26},{-30,26}}, color={0,127,255}));
    connect(bou.ports[1], hP_WaterWater_OnOff.port_a1)
      annotation (Line(points={{-106,26},{-30,26}}, color={0,127,255}));
    connect(hP_WaterWater_OnOff.port_b2, fan1.port_a)
      annotation (Line(points={{-18,26},{-2,26}}, color={0,127,255}));
    connect(hP_WaterWater_OnOff.port_a2, vol.ports[1]) annotation (Line(points=
            {{-18,6},{20,6},{20,14},{26,14}}, color={0,127,255}));
    connect(fan1.port_b, vol.ports[2]) annotation (Line(points={{18,26},{20,26},
            {20,18},{26,18}}, color={0,127,255}));
    connect(bou1.ports[1], hP_WaterWater_OnOff.port_a2) annotation (Line(points=
           {{-12,-18},{-8,-18},{-8,6},{-18,6}}, color={0,127,255}));
    connect(hP_WaterWater_OnOff.port_b1, borFie.port_a) annotation (Line(points=
           {{-30,6},{-60,6},{-60,-48},{-68,-48}}, color={0,127,255}));
    connect(hysteresis.y, not1.u)
      annotation (Line(points={{-9,80},{-14,80}}, color={255,0,255}));
    connect(not1.y, hP_WaterWater_OnOff.on) annotation (Line(points={{-37,80},{
            -40,80},{-40,18},{-34.8,18}}, color={255,0,255}));
    connect(temperatureSensor.port, vol.heatPort)
      annotation (Line(points={{50,58},{50,6},{36,6}}, color={191,0,0}));
    connect(temperatureSensor.T, hysteresis.u) annotation (Line(points={{70,58},
            {78,58},{78,82},{14,82},{14,80}}, color={0,0,127}));
    connect(booleanToReal.u, hP_WaterWater_OnOff.on) annotation (Line(points={{
            -46,60},{-40,60},{-40,18},{-34.8,18}}, color={255,0,255}));
    connect(booleanToReal.y, fan.m_flow_in) annotation (Line(points={{-69,60},{
            -148,60},{-148,-16},{-104,-16}}, color={0,0,127}));
    connect(fan1.m_flow_in, fan.m_flow_in) annotation (Line(points={{8,38},{8,
            42},{-80,42},{-80,60},{-148,60},{-148,-16},{-104,-16}}, color={0,0,
            127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Simplified_heating_system;

  model Test_mixing_Volume
     package Medium = IDEAS.Media.Water;
    IDEAS.Fluid.HeatExchangers.HeaterCooler_u hea(
      redeclare package Medium = MediumWater,
      dp_nominal=0,
      m_flow_nominal=0.1,
      Q_flow_nominal=3000)
      annotation (Placement(transformation(extent={{52,-144},{72,-124}})));
    IDEAS.Fluid.HeatExchangers.HeaterCooler_u hea1(
      redeclare package Medium = MediumWater,
      dp_nominal=0,
      m_flow_nominal=0.1,
      Q_flow_nominal=3000)
      annotation (Placement(transformation(extent={{52,-144},{72,-124}})));
    IDEAS.Fluid.HeatExchangers.HeaterCooler_u hea2(
      redeclare package Medium = MediumWater,
      dp_nominal=0,
      m_flow_nominal=0.1,
      Q_flow_nominal=3000)
      annotation (Placement(transformation(extent={{52,-144},{72,-124}})));
    IDEAS.Fluid.MixingVolumes.MixingVolume vol(
      redeclare package Medium = Medium,
      nPorts=2,
      V=10,
      T_start=308.15,
      m_flow_nominal=0)                                     annotation (
        Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=90,
          origin={-34,24})));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Test_mixing_Volume;

  model Weater_Uccle
    package Medium = IDEAS.Media.Water;
    Case900FloorHeating_thesis_Stockman case900FloorHeating_thesis_Stockman
      annotation (Placement(transformation(extent={{-32,-30},{-2,-10}})));
    IDEAS.Fluid.HeatExchangers.PrescribedOutlet preOut(use_X_wSet=false,
      redeclare package Medium = Medium,
      m_flow_nominal=1,
      dp_nominal=0)
      annotation (Placement(transformation(extent={{28,44},{48,64}})));
    IDEAS.Fluid.HeatExchangers.RadiantSlab.EmbeddedPipe embeddedPipe(
      redeclare package Medium = Medium,
      m_flow_nominal=1,
      dp_nominal=0,
      A_floor=100)
      annotation (Placement(transformation(extent={{4,24},{24,4}})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan(m_flow_nominal=1,
      redeclare package Medium = Medium,
      addPowerToMedium=false,
      inputType=IDEAS.Fluid.Types.InputType.Constant)
      annotation (Placement(transformation(extent={{-10,44},{10,64}})));
    IDEAS.Fluid.Sources.Boundary_pT bou(nPorts=1, redeclare package Medium =
          Medium)
      annotation (Placement(transformation(extent={{-56,44},{-36,64}})));
    Modelica.Blocks.Sources.Constant const(k=21)
      annotation (Placement(transformation(extent={{-56,74},{-36,94}})));
  equation
    connect(fan.port_b, preOut.port_a)
      annotation (Line(points={{10,54},{28,54}}, color={0,127,255}));
    connect(preOut.port_b, embeddedPipe.port_b) annotation (Line(points={{48,54},{
            54,54},{54,14},{24,14}}, color={0,127,255}));
    connect(embeddedPipe.port_a, fan.port_a) annotation (Line(points={{4,14},{-20,
            14},{-20,54},{-10,54}}, color={0,127,255}));
    connect(bou.ports[1], fan.port_a)
      annotation (Line(points={{-36,54},{-10,54}}, color={0,127,255}));
    connect(embeddedPipe.heatPortEmb[1], case900FloorHeating_thesis_Stockman.heatPortEmb[
      1]) annotation (Line(points={{14,4},{14,-14},{-2,-14}}, color={191,0,0}));
    connect(const.y, preOut.TSet) annotation (Line(points={{-35,84},{18,84},{18,
            62},{26,62}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Weater_Uccle;

  model Case900FloorHeating_thesis_Stockman "Case900 with floor heating"
    extends IDEAS.Templates.Structure.Case900(
      nEmb=1,
      floor(redeclare IDEAS.Buildings.Data.Constructions.InsulatedFloorHeating constructionType));
  equation
    connect(floor.port_emb[1], heatPortEmb[1]) annotation (Line(points={{-10,-14.5},
            {-4,-14.5},{-4,-30},{110,-30},{110,60},{150,60}}, color={191,0,0}));
  end Case900FloorHeating_thesis_Stockman;

  model Heating_Cooling_loads
    package Medium = IDEAS.Media.Water;
    Case900FloorHeating_thesis_Stockman case900FloorHeating_thesis_Stockman(T_start=
          294.15)
      annotation (Placement(transformation(extent={{-42,-68},{-12,-48}})));
    IDEAS.Fluid.HeatExchangers.PrescribedOutlet preOut(
      use_X_wSet=false,
      redeclare package Medium = Medium,
      m_flow_nominal=1,
      dp_nominal=0)
      annotation (Placement(transformation(extent={{-10,26},{10,46}})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan(
      m_flow_nominal=1,
      redeclare package Medium = Medium,
      addPowerToMedium=false,
      inputType=IDEAS.Fluid.Types.InputType.Constant,
      tau=60,
      constantMassFlowRate=1)
      annotation (Placement(transformation(extent={{-48,26},{-28,46}})));
    IDEAS.Fluid.Sources.Boundary_pT bou(nPorts=1, redeclare package Medium =
          Medium,
      p=200000)
      annotation (Placement(transformation(extent={{-94,26},{-74,46}})));
    Modelica.Blocks.Sources.Constant const(k=35 + 273)
      annotation (Placement(transformation(extent={{-94,56},{-74,76}})));
    IDEAS.Fluid.HeatExchangers.RadiantSlab.EmbeddedPipe embeddedPipe(
      redeclare package Medium = Medium,
      m_flow_nominal=1,
      dp_nominal=0,
      A_floor=case900FloorHeating_thesis_Stockman.floor.A)
      annotation (Placement(transformation(extent={{68,-32},{48,-52}})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan1(
      m_flow_nominal=1,
      redeclare package Medium = Medium,
      addPowerToMedium=false,
      inputType=IDEAS.Fluid.Types.InputType.Constant,
      tau=60,
      constantMassFlowRate=1)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=270,
          origin={78,-18})));
    IDEAS.Fluid.Actuators.Valves.ThreeWayLinear val(
      redeclare package Medium = Medium,
      m_flow_nominal=1,
      dpValve_nominal=1,
      CvData=IDEAS.Fluid.Types.CvTypes.OpPoint)     annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={78,8})));
    IDEAS.Controls.Continuous.LimPID conPID(
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      yMax=1,
      yMin=0,
      initType=Modelica.Blocks.Types.InitPID.NoInit,
      reverseAction=false,
      k=1,
      Ti=10) annotation (Placement(transformation(extent={{74,-82},{94,-62}})));
    Modelica.Blocks.Sources.Constant const1(k=21 + 273)
      annotation (Placement(transformation(extent={{38,-82},{58,-62}})));
    IDEAS.Fluid.MixingVolumes.MixingVolume vol(
      redeclare package Medium = Medium,
      m_flow_nominal=1,
      V=1,
      nPorts=4) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={46,34})));
  equation
    connect(fan.port_b,preOut. port_a)
      annotation (Line(points={{-28,36},{-10,36}},
                                                 color={0,127,255}));
    connect(bou.ports[1],fan. port_a)
      annotation (Line(points={{-74,36},{-48,36}}, color={0,127,255}));
    connect(const.y,preOut. TSet) annotation (Line(points={{-73,66},{-20,66},{-20,
            44},{-12,44}},color={0,0,127}));
    connect(fan1.port_a, val.port_2)
      annotation (Line(points={{78,-8},{78,-2}}, color={0,127,255}));
    connect(case900FloorHeating_thesis_Stockman.TSensor[1], conPID.u_m)
      annotation (Line(points={{-11.4,-64},{2,-64},{2,-94},{84,-94},{84,-84}},
          color={0,0,127}));
    connect(conPID.u_s, const1.y)
      annotation (Line(points={{72,-72},{59,-72}}, color={0,0,127}));
    connect(case900FloorHeating_thesis_Stockman.heatPortEmb[1], embeddedPipe.heatPortEmb[
      1]) annotation (Line(points={{-12,-52},{24,-52},{24,-52},{58,-52}}, color=
           {191,0,0}));
    connect(preOut.port_b, vol.ports[1])
      annotation (Line(points={{10,36},{36,36},{36,37}}, color={0,127,255}));
    connect(fan1.port_b, embeddedPipe.port_a) annotation (Line(points={{78,-28},
            {78,-42},{68,-42}}, color={0,127,255}));
    connect(val.port_3, embeddedPipe.port_b) annotation (Line(points={{68,8},{
            36,8},{36,-42},{48,-42}}, color={0,127,255}));
    connect(fan.port_a, vol.ports[2]) annotation (Line(points={{-48,36},{-48,12},
            {26,12},{26,35},{36,35}}, color={0,127,255}));
    connect(val.port_1, vol.ports[3]) annotation (Line(points={{78,18},{78,64},
            {36,64},{36,33}}, color={0,127,255}));
    connect(embeddedPipe.port_b, vol.ports[4])
      annotation (Line(points={{48,-42},{36,-42},{36,31}}, color={0,127,255}));
    connect(conPID.y, val.y) annotation (Line(points={{95,-72},{108,-72},{108,8},
            {90,8}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Heating_Cooling_loads;

  model Heating_cooling_loads_complete
      package Medium = IDEAS.Media.Water;
    Case900FloorHeating_thesis_Stockman case900FloorHeating_thesis_Stockman(T_start=
          294.15)
      annotation (Placement(transformation(extent={{-44,-56},{-14,-36}})));
    IDEAS.Fluid.HeatExchangers.PrescribedOutlet preOut(
      use_X_wSet=false,
      redeclare package Medium = Medium,
      m_flow_nominal=1,
      dp_nominal=0,
      tau=10,
      T_start=308.15)
      annotation (Placement(transformation(extent={{54,20},{74,40}})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan(
      m_flow_nominal=1,
      redeclare package Medium = Medium,
      addPowerToMedium=false,
      inputType=IDEAS.Fluid.Types.InputType.Constant,
      constantMassFlowRate=1,
      T_start=308.15,
      tau=10)
      annotation (Placement(transformation(extent={{-12,20},{8,40}})));
    IDEAS.Fluid.Sources.Boundary_pT bou(
      nPorts=1,
      redeclare package Medium = Medium,
      p=200000)
      annotation (Placement(transformation(extent={{-48,-2},{-28,18}})));
    IDEAS.Fluid.HeatExchangers.RadiantSlab.EmbeddedPipe embeddedPipe(
      redeclare package Medium = Medium,
      m_flow_nominal=1,
      dp_nominal=0,
      A_floor=case900FloorHeating_thesis_Stockman.floor.A,
      T_start=308.15)
      annotation (Placement(transformation(extent={{30,2},{10,-18}})));
    IDEAS.Controls.Continuous.LimPID conPID(
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      initType=Modelica.Blocks.Types.InitPID.NoInit,
      reverseAction=false,
      k=1,
      yMax=40 + 273,
      yMin=0 + 273,
      Ti=10) annotation (Placement(transformation(extent={{68,-70},{88,-50}})));
    Modelica.Blocks.Sources.Constant const1(k=21 + 273)
      annotation (Placement(transformation(extent={{36,-70},{56,-50}})));
  equation
    connect(fan.port_b,preOut. port_a)
      annotation (Line(points={{8,30},{54,30}},  color={0,127,255}));
    connect(bou.ports[1],fan. port_a)
      annotation (Line(points={{-28,8},{-28,30},{-12,30}},
                                                   color={0,127,255}));
    connect(case900FloorHeating_thesis_Stockman.TSensor[1], conPID.u_m)
      annotation (Line(points={{-13.4,-52},{0,-52},{0,-82},{78,-82},{78,-72}},
          color={0,0,127}));
    connect(conPID.u_s, const1.y)
      annotation (Line(points={{66,-60},{57,-60}}, color={0,0,127}));
    connect(case900FloorHeating_thesis_Stockman.heatPortEmb[1], embeddedPipe.heatPortEmb[
      1])
      annotation (Line(points={{-14,-40},{20,-40},{20,-18}}, color={191,0,0}));
    connect(preOut.port_b, embeddedPipe.port_a) annotation (Line(points={{74,30},
            {84,30},{84,-8},{30,-8}},color={0,127,255}));
    connect(embeddedPipe.port_b, fan.port_a)
      annotation (Line(points={{10,-8},{-12,-8},{-12,30}}, color={0,127,255}));
    connect(conPID.y, preOut.TSet) annotation (Line(points={{89,-60},{90,-60},{
            90,58},{48,58},{48,38},{52,38}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Heating_cooling_loads_complete;

  model Heating_and_Cooling
    Case900FloorHeating_thesis_Stockman case900FloorHeating_thesis_Stockman(T_start=
          294.15)
      annotation (Placement(transformation(extent={{-44,-146},{-14,-126}})));
    IDEAS.Fluid.Sources.Boundary_pT bou(
      redeclare package Medium = Medium,
      nPorts=1,
      p=200000)
      annotation (Placement(transformation(extent={{-114,28},{-94,48}})));
    IDEAS.Fluid.HeatExchangers.RadiantSlab.EmbeddedPipe embeddedPipe(
      redeclare package Medium = Medium,
      m_flow_nominal=1,
      dp_nominal=0,
      A_floor=case900FloorHeating_thesis_Stockman.floor.A)
      annotation (Placement(transformation(extent={{72,-108},{52,-128}})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan1(
      m_flow_nominal=1,
      redeclare package Medium = Medium,
      addPowerToMedium=false,
      tau=60,
      constantMassFlowRate=1,
      inputType=IDEAS.Fluid.Types.InputType.Continuous)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=270,
          origin={92,-96})));
    IDEAS.Controls.Continuous.LimPID conPID(
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      yMax=1,
      yMin=0,
      initType=Modelica.Blocks.Types.InitPID.NoInit,
      reverseAction=false,
      k=1,
      Ti=10) annotation (Placement(transformation(extent={{72,-160},{92,-140}})));
    IDEAS.Fluid.MixingVolumes.MixingVolume vol(
      redeclare package Medium = Medium,
      m_flow_nominal=1,
      V=1,
      nPorts=4) annotation (Placement(transformation(
          extent={{-13,-15},{13,15}},
          rotation=270,
          origin={49,29})));
    IDEAS.Fluid.Geothermal.Borefields.OneUTube borFie(redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater, borFieDat=
          IDEAS.Fluid.Geothermal.Borefields.Data.Borefield.Example(conDat=
          IDEAS.Fluid.Geothermal.Borefields.Data.Configuration.Example(
            borCon=IDEAS.Fluid.Geothermal.Borefields.Types.BoreholeConfiguration.SingleUTube,
            use_Rb=true,
            Rb=0.26,
            mBor_flow_nominal=0.17,
            mBorFie_flow_nominal=mBor_flow_nominal*nBor,
            rBor=0.0625,
            dBor=1,
            cooBor={{0,0},{0,6},{6,0},{6,6}},
            rTub=0.0125,
            kTub=0.43,
            eTub=0.002,
            xC=0.03)))
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-58,-78})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan2(
      m_flow_nominal=1,
      redeclare IDEAS.Fluid.Movers.Data.Generic per,
      redeclare package Medium = Medium,
      inputType=IDEAS.Fluid.Types.InputType.Constant,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      addPowerToMedium=false,
      tau=60,
      use_inputFilter=false)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-86,-54})));
    IDEAS.Fluid.HeatPumps.ScrollWaterToWater  heaPum(
      m1_flow_nominal=1,
      m2_flow_nominal=1,
      redeclare package Medium1 = Medium,
      redeclare package Medium2 = Medium)
      annotation (Placement(transformation(extent={{10,-10},{-10,10}},
          rotation=90,
          origin={-28,24})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan(
      m_flow_nominal=1,
      redeclare package Medium = Medium,
      addPowerToMedium=false,
      inputType=IDEAS.Fluid.Types.InputType.Constant,
      tau=60,
      constantMassFlowRate=1)
      annotation (Placement(transformation(extent={{-8,28},{12,48}})));
    IDEAS.Fluid.Actuators.Valves.ThreeWayLinear val1(
      redeclare package Medium = Medium,
      m_flow_nominal=1,
      dpValve_nominal=1,
      CvData=IDEAS.Fluid.Types.CvTypes.OpPoint)     annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-86,-20})));
    IDEAS.Fluid.HeatExchangers.ConstantEffectiveness hex annotation (Placement(
          transformation(
          extent={{-10,10},{10,-10}},
          rotation=270,
          origin={-8,-40})));
    IDEAS.Fluid.Actuators.Valves.ThreeWayLinear val2(
      redeclare package Medium = Medium,
      m_flow_nominal=1,
      dpValve_nominal=1,
      CvData=IDEAS.Fluid.Types.CvTypes.OpPoint)     annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={92,-30})));
    IDEAS.Fluid.Sensors.Temperature senTem annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={18,-116})));
    Modelica.Blocks.Sources.RealExpression realExpression
      annotation (Placement(transformation(extent={{18,-160},{38,-140}})));
  equation
    connect(case900FloorHeating_thesis_Stockman.heatPortEmb[1],embeddedPipe. heatPortEmb[
      1]) annotation (Line(points={{-14,-130},{22,-130},{22,-128},{62,-128}},
                                                                          color=
           {191,0,0}));
    connect(fan1.port_b,embeddedPipe. port_a) annotation (Line(points={{92,-106},
            {92,-118},{72,-118}},
                                color={0,127,255}));
    connect(embeddedPipe.port_b,vol. ports[1])
      annotation (Line(points={{52,-118},{34,-118},{34,32.9}},
                                                           color={0,127,255}));
    connect(fan2.port_a, borFie.port_b) annotation (Line(points={{-86,-64},{-86,
            -78},{-68,-78}}, color={0,127,255}));
    connect(heaPum.port_b2, fan.port_a)
      annotation (Line(points={{-22,34},{-22,38},{-8,38}}, color={0,127,255}));
    connect(heaPum.port_b1, borFie.port_a) annotation (Line(points={{-34,14},{
            -36,14},{-36,-78},{-48,-78}}, color={0,127,255}));
    connect(bou.ports[1], heaPum.port_a1) annotation (Line(points={{-94,38},{
            -34,38},{-34,34}}, color={0,127,255}));
    connect(heaPum.port_a2, vol.ports[1]) annotation (Line(points={{-22,14},{22,
            14},{22,28},{34,28},{34,32.9}},
                                          color={0,127,255}));
    connect(fan.port_b, vol.ports[2]) annotation (Line(points={{12,38},{22,38},
            {22,32},{34,32},{34,30.3}},    color={0,127,255}));
    connect(fan2.port_b, val1.port_1)
      annotation (Line(points={{-86,-44},{-86,-30}}, color={0,127,255}));
    connect(val1.port_2, heaPum.port_a1) annotation (Line(points={{-86,-10},{
            -86,38},{-34,38},{-34,34}}, color={0,127,255}));
    connect(hex.port_b1, borFie.port_a) annotation (Line(points={{-14,-50},{-14,
            -78},{-48,-78}}, color={0,127,255}));
    connect(hex.port_a1, val1.port_3) annotation (Line(points={{-14,-30},{-14,
            -20},{-76,-20}}, color={0,127,255}));
    connect(hex.port_a2, embeddedPipe.port_b) annotation (Line(points={{-2,-50},
            {34,-50},{34,-118},{52,-118}}, color={0,127,255}));
    connect(hex.port_b2, val2.port_3)
      annotation (Line(points={{-2,-30},{82,-30}}, color={0,127,255}));
    connect(val2.port_1, vol.ports[3]) annotation (Line(points={{92,-20},{92,46},
            {34,46},{34,27.7}},    color={0,127,255}));
    connect(senTem.port, vol.ports[4]) annotation (Line(points={{28,-116},{34,
            -116},{34,25.1}}, color={0,127,255}));
    connect(senTem.T, conPID.u_m) annotation (Line(points={{18,-109},{18,-100},
            {-62,-100},{-62,-176},{82,-176},{82,-162}}, color={0,0,127}));
    connect(conPID.y, fan1.m_flow_in) annotation (Line(points={{93,-150},{106,
            -150},{106,-96},{104,-96}}, color={0,0,127}));
    connect(fan1.port_a, val2.port_2)
      annotation (Line(points={{92,-86},{92,-40}}, color={0,127,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-140,
              -180},{140,100}})),                                  Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-140,-180},{140,
              100}})));
  end Heating_and_Cooling;

  model Weatherdata
    UpscaleCase900 upscaleCase900_1
      annotation (Placement(transformation(extent={{-34,6},{-4,26}})));
    HeatingCoolingSet heatingCoolingSet
      annotation (Placement(transformation(extent={{-40,34},{-20,54}})));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Weatherdata;

  model UpscaleCase900 "Upscaling of the case 900"
    extends IDEAS.Airflow.Multizone.Structure(
      final nZones=1,
      nEmb=1,
      ATrans=1,
      VZones={gF.V});
    constant Modelica.SIunits.Angle aO = 0 "Angle offset for detailed experiments";

    IDEAS.Buildings.Components.Zone gF(
      mSenFac=0.822,
      V=1200*2.7,
      n50=0.822*0.5*20,
      redeclare package Medium = Medium,
      nSurf=8,
      hZone=2.7,
      T_start=293.15)
                  annotation (Placement(transformation(extent={{40,0},{80,40}})));
    Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
      annotation (Placement(transformation(extent={{120,-70},{140,-50}})));
    IDEAS.Buildings.Components.OuterWall[4] wall(
      redeclare parameter IDEAS.Buildings.Validation.Data.Constructions.HeavyWall constructionType,
      A={108,81,48,81},
      final azi={aO+IDEAS.Types.Azimuth.N,aO+IDEAS.Types.Azimuth.E,aO+IDEAS.Types.Azimuth.S,
          aO+IDEAS.Types.Azimuth.W},
      final inc={IDEAS.Types.Tilt.Wall,IDEAS.Types.Tilt.Wall,IDEAS.Types.Tilt.Wall,
          IDEAS.Types.Tilt.Wall}) annotation (Placement(transformation(
          extent={{-5.5,-9.49999},{5.5,9.49999}},
          rotation=90,
          origin={-49.5,-14.5})));

    IDEAS.Buildings.Components.Window[2] win(
      final A={30,30},
      redeclare final parameter IDEAS.Buildings.Validation.Data.Glazing.GlaBesTest glazing,
      final inc={IDEAS.Types.Tilt.Wall,IDEAS.Types.Tilt.Wall},
      azi={aO+IDEAS.Types.Azimuth.N,aO+IDEAS.Types.Azimuth.N},
      redeclare replaceable IDEAS.Buildings.Components.Shading.None shaType,
      redeclare final parameter IDEAS.Buildings.Data.Frames.None fraType,
      each frac=0)
      annotation (Placement(transformation(
          extent={{-5.5,-9.49999},{5.5,9.49997}},
          rotation=90,
          origin={10.5,-14.5})));

    IDEAS.Buildings.Components.BoundaryWall floor(
      redeclare parameter IDEAS.Buildings.Data.Constructions.InsulatedFloorHeating constructionType,
      final A=1200,
      inc=IDEAS.Types.Tilt.Floor,
      final azi=aO+IDEAS.Types.Azimuth.S) annotation (Placement(transformation(
          extent={{-5.5,-9.5},{5.5,9.5}},
          rotation=90,
          origin={-19.5,-14.5})));
    IDEAS.Buildings.Components.OuterWall roof(
      redeclare final parameter Data.Constructions.LightRoof constructionType,
      final A=1200,
      final inc=IDEAS.Types.Tilt.Ceiling,
      final azi=aO+IDEAS.Types.Azimuth.S) annotation (Placement(transformation(
          extent={{-5.5,-9.5},{5.5,9.5}},
          rotation=90,
          origin={-79.5,-14.5})));

  equation
    connect(temperatureSensor.T, TSensor[1]) annotation (Line(
        points={{140,-60},{156,-60}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(gF.gainCon, temperatureSensor.port) annotation (Line(
        points={{80,14},{100,14},{100,-60},{120,-60}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(gF.gainCon, heatPortCon[1]) annotation (Line(
        points={{80,14},{120,14},{120,20},{150,20}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(gF.gainRad, heatPortRad[1]) annotation (Line(
        points={{80,8},{120,8},{120,-20},{150,-20}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(win.propsBus_a, gF.propsBus[7:8]) annotation (Line(
        points={{8.60001,-9.91667},{8.60001,24.5},{40,24.5}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None));

    connect(roof.propsBus_a, gF.propsBus[1]) annotation (Line(
        points={{-81.4,-9.91667},{-81.4,31.5},{40,31.5}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None));
    connect(wall.propsBus_a, gF.propsBus[2:5]) annotation (Line(
        points={{-51.4,-9.91667},{-51.4,27.5},{40,27.5}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None));
    connect(floor.propsBus_a, gF.propsBus[6]) annotation (Line(
        points={{-21.4,-9.91667},{-21.4,26.5},{40,26.5}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None));
    connect(port_b[1], gF.port_b) annotation (Line(
        points={{-20,100},{-20,60},{56,60},{56,40}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(port_a[1], gF.port_a) annotation (Line(
        points={{20,100},{20,62},{64,62},{64,40}},
        color={0,0,0},
        smooth=Smooth.None));
    connect(floor.port_emb[1], heatPortEmb[1]) annotation (Line(points={{-10,-14.5},
            {-4,-14.5},{-4,-30},{110,-30},{110,60},{150,60}}, color={191,0,0}));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-150,-100},
              {150,100}}),       graphics), Documentation(info="<html>
<p>
Basic, most generic structure of the BesTest model.
To be extended in other models.
</p>
</html>",   revisions="<html>
<ul>
<li>
March 8, 2017 by Filip Jorissen:<br/>
Added angle for offsetting building rotation.
This is for 
<a href=https://github.com/open-ideas/IDEAS/issues/689>#689</a>.
</li>
<li>
July 19, 2016 by Filip Jorissen:<br/>
First implementation
</li>
</ul>
</html>"));
  end UpscaleCase900;

  package Data

  extends Modelica.Icons.MaterialPropertiesPackage;

    package Constructions
    extends Modelica.Icons.MaterialPropertiesPackage;

      record HeavyFloor "BESTEST heavy floor"
        extends IDEAS.Buildings.Data.Interfaces.Construction(
          incLastLay = IDEAS.Types.Tilt.Floor,
          final mats={
            Thesis_Bram_Stockman.Data.Insulation.InsulationFloor(      d=1.007),
            Thesis_Bram_Stockman.Data.Materials.ConcreteSlab(      d=0.08)});

      end HeavyFloor;

      record HeavyWall "BESTEST heavy wall"
        extends IDEAS.Buildings.Data.Interfaces.Construction(
          final mats={
            Thesis_Bram_Stockman.Data.Materials.WoodSiding(      d=0.009),
            Thesis_Bram_Stockman.Data.Insulation.FoamInsulation(      d=0.0615),
            Thesis_Bram_Stockman.Data.Materials.ConcreteBlock(      d=0.10)});

      end HeavyWall;

      record HighConductance "BESTEST high conductance wall"
        extends IDEAS.Buildings.Data.Interfaces.Construction(
          final mats={
            Thesis_Bram_Stockman.Data.Materials.Glass_200(      d=0.003175),
            IDEAS.Buildings.Data.Materials.Air(d=0.013),
            Thesis_Bram_Stockman.Data.Materials.Glass_200(      d=0.003175)});

      end HighConductance;

      record LightFloor "BESTEST light floor"
        extends IDEAS.Buildings.Data.Interfaces.Construction(
          incLastLay = IDEAS.Types.Tilt.Floor,
          mats={
            Thesis_Bram_Stockman.Data.Insulation.InsulationFloor(      d=1.003),
            Thesis_Bram_Stockman.Data.Materials.TimberFlooring(      d=0.025)});

        annotation (Documentation(revisions="<html>
<ul>
<li>
July 19, 2016, Filip Jorissen:<br/>
Corrected wrong value in implementation.
</li>
</ul>
</html>"));
      end LightFloor;

      record LightRoof "BESTEST light roof"
        extends IDEAS.Buildings.Data.Interfaces.Construction(
          incLastLay = IDEAS.Types.Tilt.Ceiling,
          final mats={
            Thesis_Bram_Stockman.Data.Materials.Roofdeck(      d=0.019),
            Thesis_Bram_Stockman.Data.Insulation.FiberGlass(      d=0.1118),
            Thesis_Bram_Stockman.Data.Materials.PlasterBoard(      d=0.010)});

      end LightRoof;

      record LightRoof_195 "BESTEST light roof for case 195"
        extends IDEAS.Buildings.Data.Interfaces.Construction(
          incLastLay = IDEAS.Types.Tilt.Ceiling,
          final mats={
            Thesis_Bram_Stockman.Data.Materials.Roofdeck_195(      d=0.019),
            Thesis_Bram_Stockman.Data.Insulation.FiberGlass(      d=0.1118),
            Thesis_Bram_Stockman.Data.Materials.PlasterBoard(      d=0.010)});

      end LightRoof_195;

      record LightWall "BESTEST light wall"
        extends IDEAS.Buildings.Data.Interfaces.Construction(
          final mats={
            Thesis_Bram_Stockman.Data.Materials.WoodSiding(      d=0.009),
            Thesis_Bram_Stockman.Data.Insulation.FiberGlass(      d=0.066),
            Thesis_Bram_Stockman.Data.Materials.PlasterBoard(      d=0.012)});
      end LightWall;

      record LightWall_195 "BESTEST light wall for case 195"
        extends IDEAS.Buildings.Data.Interfaces.Construction(
          final mats={
              Thesis_Bram_Stockman.Data.Materials.WoodSiding_195(      d=0.009),
              Thesis_Bram_Stockman.Data.Insulation.FiberGlass(      d=0.066),
              Thesis_Bram_Stockman.Data.Materials.PlasterBoard(      d=0.012)});

      end LightWall_195;
    end Constructions;

    package Glazing
    extends Modelica.Icons.MaterialPropertiesPackage;

      record GlaBesTest = IDEAS.Buildings.Data.Interfaces.Glazing (
          final nLay=3,
          final mats={IDEAS.Buildings.Data.Materials.Glass(
                d=0.003175,
                epsLw=0.9,
                k=1.06),
              IDEAS.Buildings.Data.Materials.Air(d=0.013),
              IDEAS.Buildings.Data.Materials.Glass(
                d=0.003175,
                epsLw=0.9,
                k=1.06)},
          final SwTrans=[
                0, 0.747454;
                10, 0.746824;
                20, 0.744654;
                30, 0.739491;
                40, 0.729832;
                45, 0.720922;
                50, 0.707331;
                60, 0.652331;
                70, 0.516754;
                80, 0.263009;
                88.9, 0.00;
                90, 0.00],
          final SwAbs=[
              0,  0.0643, 0.0, 0.0522;
              20, 0.0659, 0.0, 0.0534;
              30, 0.0679, 0.0, 0.0548;
              40, 0.0708, 0.0, 0.0566;
              48, 0.0738, 0.0, 0.058;
              55, 0.0769, 0.0, 0.0587;
              57, 0.0779, 0.0, 0.0587;
              60, 0.0796, 0.0, 0.0585;
              63, 0.0815, 0.0, 0.0579;
              66, 0.0837, 0.0, 0.0568;
              68, 0.0852, 0.0, 0.0558;
              70, 0.0858, 0.0, 0.0544;
              72, 0.089,  0.0, 0.0521;
              75, 0.0911, 0.0, 0.0495;
              77.5, 0.0929, 0.0, 0.0457;
              80, 0.094, 0.0, 0.0413;
              82, 0.0937, 0.0, 0.0372;
              83.5, 0.0924, 0.0, 0.0335;
              85, 0.0892, 0.0, 0.0291;
              86, 0.0854, 0.0, 0.0254;
              87, 0.079,  0.0, 0.0205;
              88, 0.0671, 0.0, 0.0128;
              89, 0.0473, 0.0, 0.0043;
              89.5, 0.304, 0.0, 0.0004;
              89.99, 0.001,0.0, 0.0;
              90, 0.00, 0.0, 0.0],
          final U_value=3.0,
          final g_value=0.87,
          final SwAbsDif={0.0796,0.0,0.0585},
          final SwTransDif=0.652331) "BESTEST glazing";
    end Glazing;

    package Insulation
    extends Modelica.Icons.MaterialPropertiesPackage;

      record FiberGlass =
        IDEAS.Buildings.Data.Interfaces.Insulation (
          final k=0.040,
          final c=840,
          final rho=12,
          epsLw=0.9,
          epsSw=0.6) "BESTEST fiberglass insulation";
      record FoamInsulation =
        IDEAS.Buildings.Data.Interfaces.Insulation (
          final k=0.040,
          final c=1400,
          final rho=10,
          epsLw=0.9,
          epsSw=0.6) "BESTEST foam insulation";
      record InsulationFloor =
        IDEAS.Buildings.Data.Interfaces.Insulation (
          final k=0.040,
          final c=10,
          final rho=10,
          epsLw=0.9,
          epsSw=0.6) "BESTEST floor insulation";
    end Insulation;

    package Materials
    extends Modelica.Icons.MaterialPropertiesPackage;

      record ConcreteBlock
        "BESTEST concrete blocks"
        extends IDEAS.Buildings.Data.Interfaces.Material(
          final k=0.51,
          final c=1000,
          final rho=1400,
          epsLw=0.9,
          epsSw=0.6);
      end ConcreteBlock;

      record ConcreteSlab
        "BESTEST concrete slab"
        extends IDEAS.Buildings.Data.Interfaces.Material(
          final k=1.130,
          final c=1000,
          final rho=1400,
          epsLw=0.9,
          epsSw=0.6);
      end ConcreteSlab;

      record Glass_200
        "BESTEST glass"
        extends IDEAS.Buildings.Data.Interfaces.Material(
          k=0.96,
          c=750,
          rho=2500,
          epsLw=0.1,
          epsSw=0.1);

      end Glass_200;

      record PlasterBoard
        "BESTEST plasterboard"
        extends IDEAS.Buildings.Data.Interfaces.Material(
          final k=0.160,
          final c=840,
          final rho=950,
          epsLw=0.9,
          epsSw=0.6);
      end PlasterBoard;

      record Roofdeck
        "BESTEST roof deck"
        extends IDEAS.Buildings.Data.Interfaces.Material(
          final k=0.14,
          final c=900,
          final rho=530,
          epsLw=0.9,
          epsSw=0.6);
      end Roofdeck;

      record Roofdeck_195
        "BESTEST roof deck for case 195"
        extends IDEAS.Buildings.Data.Interfaces.Material(
          final k=0.14,
          final c=900,
          final rho=530,
          epsLw=0.1,
          epsSw=0.1);
      end Roofdeck_195;

      record TimberFlooring
        "BESTEST timber flooring"
        extends IDEAS.Buildings.Data.Interfaces.Material(
          final k=0.14,
          final c=1200,
          final rho=650,
          epsLw=0.9,
          epsSw=0.6);
      end TimberFlooring;

      record WoodSiding
        "BESTEST wood siding"
        extends IDEAS.Buildings.Data.Interfaces.Material(
          final k=0.14,
          final c=900,
          final rho=530,
          epsLw=0.9,
          epsSw=0.6);
      end WoodSiding;

      record WoodSiding_195
        "BESTEST wood siding for case 195"
        extends IDEAS.Buildings.Data.Interfaces.Material(
          final k=0.14,
          final c=900,
          final rho=530,
          epsLw=0.1,
          epsSw=0.1);
      end WoodSiding_195;
    end Materials;
  end Data;

  model RBC
    package Medium = IDEAS.Media.Water;
    UpscaleCase900 upscaleCase900_HVAC
      annotation (Placement(transformation(extent={{-58,-182},{-28,-162}})));
    IDEAS.Fluid.HeatExchangers.RadiantSlab.EmbeddedPipe embeddedPipe(
      redeclare package Medium = Medium,
      A_floor=1200,
      redeclare IDEAS.Fluid.HeatExchangers.RadiantSlab.BaseClasses.FH_Standard1
        RadSlaCha,
      m_flow_nominal=1,
      T_start=308.15)
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={42,-98})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fanEmbeddedPipe(
      redeclare package Medium = Medium,
      addPowerToMedium=false,
      tau=60,
      constantMassFlowRate=1,
      inputType=IDEAS.Fluid.Types.InputType.Continuous,
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      use_inputFilter=false,
      m_flow_nominal=1,
      T_start=308.15) annotation (Placement(transformation(
          extent={{-8,-8},{8,8}},
          rotation=270,
          origin={72,-66})));
    IDEAS.Fluid.MixingVolumes.MixingVolume BufferTank(
      redeclare package Medium = Medium,
      nPorts=5,
      m_flow_nominal=fanBorefield.m_flow_nominal,
      V=2,
      T_start=308.15) annotation (Placement(transformation(
          extent={{-9,-8},{9,8}},
          rotation=0,
          origin={43,64})));
    IDEAS.Fluid.Actuators.Valves.Simplified.ThreeWayValveSwitch SwitchHeatCoolEmbeddedPipe(
      redeclare package Medium = Medium,
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      m_flow_nominal=1,
      T_start=308.15) annotation (Placement(transformation(
          extent={{-8,-8},{8,8}},
          rotation=270,
          origin={76,18})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fanBuffertank(
      redeclare package Medium = Medium,
      addPowerToMedium=false,
      tau=60,
      constantMassFlowRate=1,
      inputType=IDEAS.Fluid.Types.InputType.Continuous,
      use_inputFilter=false,
      m_flow_nominal=2,
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      T_start=308.15) annotation (Placement(transformation(
          extent={{-8,-8},{8,8}},
          rotation=0,
          origin={6,52})));
    IDEAS.Fluid.HeatPumps.ScrollWaterToWater heaPum(
      redeclare package Medium1 = Medium,
      redeclare package Medium2 = Medium,
      allowFlowReversal1=false,
      allowFlowReversal2=false,
      redeclare package ref = IDEAS.Media.Refrigerants.R410A,
      datHeaPum=
          IDEAS.Fluid.HeatPumps.Data.ScrollWaterToWater.Heating.ClimateMaster_TMW036_12kW_4_90COP_R410A(),

      m1_flow_nominal=fanBuffertank.m_flow_nominal,
      m2_flow_nominal=fanBorefield.m_flow_nominal,
      enable_variable_speed=false,
      dp1_nominal=0,
      dp2_nominal=0,
      scaling_factor=3) annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=270,
          origin={-20,42})));

    IDEAS.Fluid.Geothermal.Borefields.OneUTube borFie(redeclare package Medium =
          Medium, borFieDat=
          IDEAS.Fluid.Geothermal.Borefields.Data.Borefield.Example(
            filDat=IDEAS.Fluid.Geothermal.Borefields.Data.Filling.Bentonite(
              kFil=0.6,
              cFil=1650,
              dFil=1000,
              steadyState=true),
            soiDat=IDEAS.Fluid.Geothermal.Borefields.Data.Soil.SandStone(
              kSoi=2.2,
              cSoi=2470,
              dSoi=1000),
            conDat=IDEAS.Fluid.Geothermal.Borefields.Data.Configuration.Example(
              borCon=IDEAS.Fluid.Geothermal.Borefields.Types.BoreholeConfiguration.SingleUTube,
              use_Rb=true,
              Rb=0.266,
              mBor_flow_nominal=6.5/18,
              mBorFie_flow_nominal=6.5,
              hBor=100,
              dBor=1,
              nBor=18,
              cooBor={{0,0},{0,6},{6,0},{6,6},{0,12},{6,12},{12,12},{12,6},{12,
              0},{0,18},{6,18},{12,18},{18,18},{18,12},{18,6},{18,0},{0,24},{6,
              24}},
              kTub=0.38)))
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-60,-28})));
    IDEAS.Fluid.HeatExchangers.ConstantEffectiveness hex(
      redeclare package Medium1 = Medium,
      redeclare package Medium2 = Medium,
      eps=0.8,
      m1_flow_nominal=fanBorefield.m_flow_nominal,
      dp1_nominal=0,
      dp2_nominal=0,
      m2_flow_nominal=1) annotation (Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=270,
          origin={8,8})));
    IDEAS.Fluid.Actuators.Valves.Simplified.ThreeWayValveSwitch
      SwitchHeatCoolBorefield(
      redeclare package Medium = Medium,
      m_flow_nominal=fanBorefield.m_flow_nominal,
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState) annotation (
        Placement(transformation(
          extent={{8,-8},{-8,8}},
          rotation=90,
          origin={-78,18})));
    IDEAS.Fluid.Sources.Boundary_pT bou(
      redeclare package Medium = Medium,
      nPorts=1,
      p=150000)
      annotation (Placement(transformation(extent={{-98,52},{-84,66}})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fanBorefield(
      redeclare IDEAS.Fluid.Movers.Data.Generic per,
      redeclare package Medium = Medium,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      addPowerToMedium=false,
      tau=60,
      use_inputFilter=false,
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      inputType=IDEAS.Fluid.Types.InputType.Continuous,
      m_flow_nominal=2) annotation (Placement(transformation(
          extent={{-9,-8},{9,8}},
          rotation=90,
          origin={-78,-5})));
    IDEAS.Fluid.Sources.Boundary_pT bou1(
      redeclare package Medium = Medium,
      nPorts=1,
      p=150000)
      annotation (Placement(transformation(extent={{-7,-7},{7,7}},
          rotation=270,
          origin={77,77})));
    Modelica.Blocks.Logical.Hysteresis
                                     hysteresis(
      pre_y_start=false,
      uLow=273.15 + 32,
      uHigh=273.15 + 37)
             annotation (Placement(transformation(extent={{128,26},{146,44}})));
    Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
      annotation (Placement(transformation(extent={{34,80},{48,94}})));
    Modelica.Blocks.Math.RealToBoolean Heating
      annotation (Placement(transformation(extent={{-288,114},{-276,126}})));
    Modelica.Blocks.Logical.Not not1
      annotation (Placement(transformation(extent={{-240,114},{-228,126}})));
    Modelica.Blocks.Math.BooleanToReal booleanToReal
      annotation (Placement(transformation(extent={{262,20},{276,34}})));
    Modelica.Blocks.Logical.Not not2
      annotation (Placement(transformation(extent={{160,28},{174,42}})));
    Modelica.Blocks.Logical.And and1
      annotation (Placement(transformation(extent={{198,20},{212,34}})));
    Modelica.Blocks.Math.RealToInteger realToInteger1
      annotation (Placement(transformation(extent={{290,20},{304,34}})));
    Modelica.Blocks.Logical.Or or1
      annotation (Placement(transformation(extent={{-190,-12},{-176,2}})));
    Modelica.Blocks.Math.Gain gain(k=2) annotation (Placement(transformation(
          extent={{-8,-8},{8,8}},
          rotation=270,
          origin={6,82})));
    Modelica.Blocks.Math.BooleanToReal booleanToReal1
      annotation (Placement(transformation(extent={{-168,-12},{-154,2}})));
    Modelica.Blocks.Math.Gain gain1(k=2) annotation (Placement(transformation(
          extent={{-7,-7},{7,7}},
          rotation=0,
          origin={-137,-5})));
    IDEAS.Fluid.Sensors.TemperatureTwoPort EmbeddedPipeIn(
      redeclare package Medium = Medium,
      tau=0,
      initType=Modelica.Blocks.Types.Init.SteadyState,
      m_flow_nominal=1,
      T_start=308.15)
      annotation (Placement(transformation(extent={{70,-92},{56,-104}})));
    IDEAS.Fluid.Sensors.TemperatureTwoPort EmbeddedPipeOut(
      redeclare package Medium = Medium,
      tau=0,
      initType=Modelica.Blocks.Types.Init.SteadyState,
      m_flow_nominal=1,
      T_start=308.15)
      annotation (Placement(transformation(extent={{30,-92},{16,-104}})));
    IDEAS.Fluid.Actuators.Valves.Simplified.ThreeWayValveMotor threeWayValveTempSetoint(
      redeclare package Medium = Medium,
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      allowFlowReversal=true,
      m_flow_nominal=1,
      T_start=308.15) annotation (Placement(transformation(
          extent={{-8,8},{8,-8}},
          rotation=270,
          origin={20,-38})));
    IDEAS.Controls.Continuous.LimPID conPIDOutHeat(
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      xi_start=0,
      yMax=1,
      reverseAction=false,
      k=1,
      Ti=60,
      yMin=0.1)
      annotation (Placement(transformation(extent={{166,-148},{182,-132}})));
    IDEAS.Controls.Continuous.LimPID conPIDINHeat(
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      yMax=1,
      xi_start=1,
      k=1,
      yMin=0,
      Ti=30)
      annotation (Placement(transformation(extent={{-90,-78},{-74,-62}})));
    Modelica.Blocks.Math.Gain gain2(k=1) annotation (Placement(transformation(
          extent={{-7,-7},{7,7}},
          rotation=90,
          origin={191,-87})));
    RunningMeanTemperature6h runningMeanTemperature6h
      annotation (Placement(transformation(extent={{-184,-78},{-166,-58}})));
    IDEAS.Controls.SetPoints.Table tab(table=[-8.0 + 273,40 + 273; 15 + 273,27
           + 273])
      annotation (Placement(transformation(extent={{-136,-78},{-116,-58}})));
    IDEAS.Controls.SetPoints.Table tab1(table=[-8 + 273,35 + 273; 10 + 273,27
           + 273])
      annotation (Placement(transformation(extent={{134,-150},{154,-130}})));
    IDEAS.Fluid.FixedResistances.Junction jun(
      redeclare package Medium = Medium,
      dp_nominal={0,0,0},
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      T_start=308.15,
      m_flow_nominal={1,1,1})
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=270,
          origin={76,-16})));
    IDEAS.Fluid.FixedResistances.Junction jun1(
      redeclare package Medium = Medium,
      m_flow_nominal={6,6,6},
      dp_nominal={0,0,0},
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-28,-28})));
    IDEAS.Fluid.Sensors.TemperatureTwoPort HexOut(
      redeclare package Medium = Medium,
      tau=0,
      initType=Modelica.Blocks.Types.Init.SteadyState,
      m_flow_nominal=1,
      T_start=308.15)
      annotation (Placement(transformation(extent={{36,24},{22,12}})));
    RunningMeanTemperature6h runningMeanTemperature6h1
      annotation (Placement(transformation(extent={{100,-150},{118,-130}})));
    IDEAS.Controls.SetPoints.Table tab2(table=[17 + 273,24 + 273; 35 + 273,22
           + 273])
      annotation (Placement(transformation(extent={{132,-190},{152,-170}})));
    RunningMeanTemperature6h runningMeanTemperature6h2
      annotation (Placement(transformation(extent={{100,-190},{118,-170}})));
    IDEAS.Controls.Continuous.LimPID conPIDOutCool(
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      xi_start=0,
      Ti=60,
      k=1,
      yMax=1,
      yMin=0.1,
      reverseAction=true)
      annotation (Placement(transformation(extent={{164,-188},{180,-172}})));
    Modelica.Blocks.Logical.Switch switch1
      annotation (Placement(transformation(extent={{200,-172},{220,-152}})));
    IDEAS.Controls.SetPoints.Table tab3(table=[15 + 273,24 + 273; 35 + 273,17
           + 273])
      annotation (Placement(transformation(extent={{-136,-130},{-116,-110}})));
    IDEAS.Controls.Continuous.LimPID conPIDINCool(
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      yMax=1,
      xi_start=1,
      k=1,
      yMin=0,
      Ti=30,
      reverseAction=true)
      annotation (Placement(transformation(extent={{-90,-128},{-74,-112}})));
    Modelica.Blocks.Logical.Switch switch2
      annotation (Placement(transformation(extent={{-60,-106},{-40,-86}})));
    HeatingCoolingSet heatingCoolingSet1
      annotation (Placement(transformation(extent={{-332,88},{-312,108}})));
    Modelica.Blocks.Math.RealToBoolean Cooling
      annotation (Placement(transformation(extent={{-292,70},{-278,84}})));
    Modelica.Blocks.Logical.Or  Neutral
      annotation (Placement(transformation(extent={{-240,92},{-228,104}})));
    Modelica.Blocks.Logical.Switch switch3
      annotation (Placement(transformation(extent={{-120,-34},{-104,-18}})));
    Modelica.Blocks.Sources.Constant const(k=0)
      annotation (Placement(transformation(extent={{-220,-50},{-206,-36}})));
    Modelica.Blocks.Logical.LogicalSwitch logicalSwitch
      annotation (Placement(transformation(extent={{230,-6},{246,10}})));
    Modelica.Blocks.Sources.BooleanConstant booleanConstant(k=false)
      annotation (Placement(transformation(extent={{202,-26},{216,-12}})));
    Modelica.Blocks.Logical.Switch switch4
      annotation (Placement(transformation(extent={{126,-74},{110,-58}})));
    Modelica.Blocks.Sources.Constant const1(k=0.01)
      annotation (Placement(transformation(extent={{154,-102},{138,-86}})));
  equation
    connect(upscaleCase900_HVAC.heatPortEmb, embeddedPipe.heatPortEmb)
      annotation (Line(points={{-28,-166},{42,-166},{42,-108}},
                                                             color={191,0,0}));
    connect(fanBuffertank.port_b, BufferTank.ports[2]) annotation (Line(points=
            {{14,52},{41.56,52},{41.56,56}}, color={0,127,255}));
    connect(fanBuffertank.port_a, heaPum.port_b1)
      annotation (Line(points={{-2,52},{-14,52}}, color={0,127,255}));
    connect(heaPum.port_a1, BufferTank.ports[3])
      annotation (Line(points={{-14,32},{43,32},{43,56}}, color={0,127,255}));
    connect(bou.ports[1], heaPum.port_a2)
      annotation (Line(points={{-84,59},{-78,59},{-78,52},{-26,52}},
                                                   color={0,127,255}));
    connect(fanBorefield.port_a, borFie.port_b) annotation (Line(points={{-78,
            -14},{-78,-28},{-70,-28}}, color={0,127,255}));
    connect(SwitchHeatCoolBorefield.port_b, fanBorefield.port_b)
      annotation (Line(points={{-78,10},{-78,4}}, color={0,127,255}));
    connect(SwitchHeatCoolBorefield.port_a1, heaPum.port_a2) annotation (Line(
          points={{-78,26},{-78,52},{-26,52}}, color={0,127,255}));
    connect(SwitchHeatCoolBorefield.port_a2, hex.port_a1)
      annotation (Line(points={{-70,18},{2,18}}, color={0,127,255}));
    connect(temperatureSensor.port, BufferTank.heatPort) annotation (Line(
          points={{34,87},{28,87},{28,64},{34,64}}, color={191,0,0}));
    connect(SwitchHeatCoolBorefield.switch, SwitchHeatCoolEmbeddedPipe.switch)
      annotation (Line(points={{-84.4,18},{-116,18},{-116,120},{88,120},{88,18},
            {82.4,18}}, color={255,0,255}));
    connect(Heating.y, not1.u)
      annotation (Line(points={{-275.4,120},{-241.2,120}}, color={255,0,255}));
    connect(not1.y, SwitchHeatCoolEmbeddedPipe.switch) annotation (Line(points=
            {{-227.4,120},{88,120},{88,18},{82.4,18}}, color={255,0,255}));
    connect(not2.u, hysteresis.y)
      annotation (Line(points={{158.6,35},{146.9,35}}, color={255,0,255}));
    connect(temperatureSensor.T, hysteresis.u) annotation (Line(points={{48,87},
            {108,87},{108,35},{126.2,35}},
                                         color={0,0,127}));
    connect(realToInteger1.y, heaPum.stage) annotation (Line(points={{304.7,27},
            {310,27},{310,114},{-40,114},{-40,24},{-17,24},{-17,30}}, color={
            255,127,0}));
    connect(gain.y, fanBuffertank.m_flow_in)
      annotation (Line(points={{6,73.2},{6,61.6}}, color={0,0,127}));
    connect(gain.u, booleanToReal.y) annotation (Line(points={{6,91.6},{6,104},
            {284,104},{284,27},{276.7,27}}, color={0,0,127}));
    connect(realToInteger1.u, booleanToReal.y)
      annotation (Line(points={{288.6,27},{276.7,27}}, color={0,0,127}));
    connect(booleanToReal1.u, or1.y)
      annotation (Line(points={{-169.4,-5},{-175.3,-5}}, color={255,0,255}));
    connect(booleanToReal1.y, gain1.u)
      annotation (Line(points={{-153.3,-5},{-145.4,-5}}, color={0,0,127}));
    connect(or1.u1, booleanToReal.u) annotation (Line(points={{-191.4,-5},{-194,
            -5},{-194,-4},{-208,-4},{-208,108},{322,108},{322,16},{254,16},{254,
            27},{260.6,27}},
          color={255,0,255}));
    connect(embeddedPipe.port_a, EmbeddedPipeIn.port_b)
      annotation (Line(points={{52,-98},{56,-98}}, color={0,127,255}));
    connect(fanEmbeddedPipe.port_b, EmbeddedPipeIn.port_a) annotation (Line(
          points={{72,-74},{72,-98},{70,-98}}, color={0,127,255}));
    connect(embeddedPipe.port_b, EmbeddedPipeOut.port_a)
      annotation (Line(points={{32,-98},{30,-98}}, color={0,127,255}));
    connect(threeWayValveTempSetoint.port_b, EmbeddedPipeOut.port_b)
      annotation (Line(points={{20,-46},{20,-80},{12,-80},{12,-98},{16,-98}},
          color={0,127,255}));
    connect(runningMeanTemperature6h.TRm, tab.u)
      annotation (Line(points={{-165.46,-68},{-138,-68}},
                                                        color={0,0,127}));
    connect(BufferTank.ports[4], SwitchHeatCoolEmbeddedPipe.port_a1)
      annotation (Line(points={{44.44,56},{76,56},{76,26}}, color={0,127,255}));
    connect(bou1.ports[1], SwitchHeatCoolEmbeddedPipe.port_a1)
      annotation (Line(points={{77,70},{76,70},{76,26}}, color={0,127,255}));
    connect(EmbeddedPipeIn.T, conPIDINHeat.u_m) annotation (Line(points={{63,
            -104.6},{64,-104.6},{64,-128},{-22,-128},{-22,-79.6},{-82,-79.6}},
          color={0,0,127}));
    connect(EmbeddedPipeOut.T, conPIDOutHeat.u_m) annotation (Line(points={{23,
            -104.6},{23,-198},{76,-198},{76,-156},{174,-156},{174,-149.6}},
          color={0,0,127}));
    connect(threeWayValveTempSetoint.port_a2, jun.port_3) annotation (Line(
          points={{28,-38},{58,-38},{58,-16},{66,-16}}, color={0,127,255}));
    connect(SwitchHeatCoolEmbeddedPipe.port_b, jun.port_1)
      annotation (Line(points={{76,10},{76,-6}}, color={0,127,255}));
    connect(fanEmbeddedPipe.port_a, jun.port_2) annotation (Line(points={{72,
            -58},{72,-50},{76,-50},{76,-26}}, color={0,127,255}));
    connect(heaPum.port_b2, jun1.port_3) annotation (Line(points={{-26,32},{-28,
            32},{-28,-18}}, color={0,127,255}));
    connect(borFie.port_a, jun1.port_2)
      annotation (Line(points={{-50,-28},{-38,-28}}, color={0,127,255}));
    connect(hex.port_b1, jun1.port_1) annotation (Line(points={{2,-2},{-8,-2},{
            -8,-28},{-18,-28}}, color={0,127,255}));
    connect(threeWayValveTempSetoint.port_a1, hex.port_a2) annotation (Line(
          points={{20,-30},{18,-30},{18,-2},{14,-2}}, color={0,127,255}));
    connect(threeWayValveTempSetoint.port_a1, BufferTank.ports[5]) annotation (
        Line(points={{20,-30},{45.88,-30},{45.88,56}}, color={0,127,255}));
    connect(tab1.y, conPIDOutHeat.u_s)
      annotation (Line(points={{155,-140},{164.4,-140}}, color={0,0,127}));
    connect(HexOut.port_b, hex.port_b2)
      annotation (Line(points={{22,18},{14,18}}, color={0,127,255}));
    connect(HexOut.port_a, SwitchHeatCoolEmbeddedPipe.port_a2)
      annotation (Line(points={{36,18},{68,18}}, color={0,127,255}));
    connect(tab1.u, runningMeanTemperature6h1.TRm)
      annotation (Line(points={{132,-140},{118.54,-140}}, color={0,0,127}));
    connect(tab2.u, runningMeanTemperature6h2.TRm)
      annotation (Line(points={{130,-180},{118.54,-180}}, color={0,0,127}));
    connect(tab2.y, conPIDOutCool.u_s)
      annotation (Line(points={{153,-180},{162.4,-180}}, color={0,0,127}));
    connect(conPIDOutHeat.y, switch1.u1) annotation (Line(points={{182.8,-140},
            {188,-140},{188,-154},{198,-154}}, color={0,0,127}));
    connect(conPIDOutCool.y, switch1.u3) annotation (Line(points={{180.8,-180},
            {188,-180},{188,-170},{198,-170}}, color={0,0,127}));
    connect(Heating.y, switch1.u2) annotation (Line(points={{-275.4,120},{-268,
            120},{-268,-204},{82,-204},{82,-162},{198,-162}}, color={255,0,255}));
    connect(conPIDOutCool.u_m, conPIDOutHeat.u_m) annotation (Line(points={{172,
            -189.6},{172,-210},{76,-210},{76,-156},{174,-156},{174,-149.6}},
          color={0,0,127}));
    connect(switch1.y, gain2.u) annotation (Line(points={{221,-162},{228,-162},
            {228,-108},{191,-108},{191,-95.4}}, color={0,0,127}));
    connect(tab3.u, tab.u) annotation (Line(points={{-138,-120},{-152,-120},{
            -152,-68},{-138,-68}}, color={0,0,127}));
    connect(tab.y, conPIDINHeat.u_s) annotation (Line(points={{-115,-68},{-104,
            -68},{-104,-70},{-91.6,-70}}, color={0,0,127}));
    connect(tab3.y, conPIDINCool.u_s)
      annotation (Line(points={{-115,-120},{-91.6,-120}}, color={0,0,127}));
    connect(conPIDINHeat.y, switch2.u1) annotation (Line(points={{-73.2,-70},{-68,
            -70},{-68,-88},{-62,-88}}, color={0,0,127}));
    connect(conPIDINCool.y, switch2.u3) annotation (Line(points={{-73.2,-120},{
            -68,-120},{-68,-104},{-62,-104}}, color={0,0,127}));
    connect(conPIDINCool.u_m, conPIDINHeat.u_m) annotation (Line(points={{-82,
            -129.6},{-66,-129.6},{-66,-142},{-20,-142},{-20,-79.6},{-82,-79.6}},
          color={0,0,127}));
    connect(switch2.u2, switch1.u2) annotation (Line(points={{-62,-96},{-204,
            -96},{-204,-204},{82,-204},{82,-162},{198,-162}}, color={255,0,255}));
    connect(switch2.y, threeWayValveTempSetoint.ctrl) annotation (Line(points={
            {-39,-96},{-4,-96},{-4,-38},{11.36,-38}}, color={0,0,127}));
    connect(heatingCoolingSet1.SetpointCooling, Cooling.u) annotation (Line(
          points={{-311,95.6},{-306,95.6},{-306,96},{-300,96},{-300,77},{-293.4,
            77}}, color={0,0,127}));
    connect(Cooling.y, Neutral.u2) annotation (Line(points={{-277.3,77},{-252,
            77},{-252,93.2},{-241.2,93.2}}, color={255,0,255}));
    connect(heatingCoolingSet1.SetpointHeating, Heating.u) annotation (Line(
          points={{-311,98.8},{-300,98.8},{-300,120},{-289.2,120}}, color={0,0,
            127}));
    connect(Neutral.u1, Heating.y) annotation (Line(points={{-241.2,98},{-268,
            98},{-268,120},{-275.4,120}}, color={255,0,255}));
    connect(and1.u2, not1.u) annotation (Line(points={{196.6,21.4},{98,21.4},{
            98,140},{-268,140},{-268,120},{-241.2,120}}, color={255,0,255}));
    connect(or1.u2, SwitchHeatCoolEmbeddedPipe.switch) annotation (Line(points=
            {{-191.4,-10.6},{-216,-10.6},{-216,120},{88,120},{88,18},{82.4,18}},
          color={255,0,255}));
    connect(not2.y, and1.u1) annotation (Line(points={{174.7,35},{184.35,35},{
            184.35,27},{196.6,27}}, color={255,0,255}));
    connect(gain1.y, switch3.u1) annotation (Line(points={{-129.3,-5},{-126,-5},
            {-126,-19.6},{-121.6,-19.6}}, color={0,0,127}));
    connect(switch3.y, fanBorefield.m_flow_in) annotation (Line(points={{-103.2,
            -26},{-87.6,-26},{-87.6,-5}}, color={0,0,127}));
    connect(Neutral.y, switch3.u2) annotation (Line(points={{-227.4,98},{-222,
            98},{-222,-26},{-121.6,-26}}, color={255,0,255}));
    connect(const.y, switch3.u3) annotation (Line(points={{-205.3,-43},{-163.65,
            -43},{-163.65,-32.4},{-121.6,-32.4}}, color={0,0,127}));
    connect(logicalSwitch.y, booleanToReal.u) annotation (Line(points={{246.8,2},
            {246,2},{246,27},{260.6,27}}, color={255,0,255}));
    connect(and1.y, logicalSwitch.u1) annotation (Line(points={{212.7,27},{
            212.7,17.5},{228.4,17.5},{228.4,8.4}}, color={255,0,255}));
    connect(logicalSwitch.u2, switch3.u2) annotation (Line(points={{228.4,2},{
            92,2},{92,132},{-222,132},{-222,-26},{-121.6,-26}}, color={255,0,
            255}));
    connect(logicalSwitch.u3, booleanConstant.y) annotation (Line(points={{
            228.4,-4.4},{220,-4.4},{220,-19},{216.7,-19}}, color={255,0,255}));
    connect(gain2.y, switch4.u1) annotation (Line(points={{191,-79.3},{191,
            -59.6},{127.6,-59.6}}, color={0,0,127}));
    connect(switch4.y, fanEmbeddedPipe.m_flow_in)
      annotation (Line(points={{109.2,-66},{81.6,-66}}, color={0,0,127}));
    connect(switch4.u3, const1.y) annotation (Line(points={{127.6,-72.4},{127.6,
            -94},{137.2,-94}}, color={0,0,127}));
    connect(switch4.u2, switch3.u2) annotation (Line(points={{127.6,-66},{154,
            -66},{154,2},{92,2},{92,132},{-222,132},{-222,-26},{-121.6,-26}},
          color={255,0,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-280,
              -220},{280,160}})),                                  Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-280,-220},{280,
              160}})),
      experiment(
        StopTime=31536000,
        __Dymola_NumberOfIntervals=15000,
        Tolerance=1e-06,
        __Dymola_fixedstepsize=20,
        __Dymola_Algorithm="Euler"),
      __Dymola_experimentSetupOutput,
      __Dymola_experimentFlags(
        Advanced(GenerateVariableDependencies=false, OutputModelicaCode=false),
        Evaluate=false,
        OutputCPUtime=false,
        OutputFlatModelica=false));
  end RBC;

  block HeatingCoolingMode
    "Block that computes if the system is in heating or cooling mode"
    HeatingCoolingSet heatingCoolingSet
      annotation (Placement(transformation(extent={{-84,18},{-64,38}})));
    Modelica.Blocks.Math.RealToBoolean Heating
      annotation (Placement(transformation(extent={{-50,44},{-30,64}})));
    Modelica.Blocks.Math.RealToBoolean Cooling
      annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
    Modelica.Blocks.Logical.Nor Neutral
      annotation (Placement(transformation(extent={{-18,20},{2,40}})));
  equation
    connect(heatingCoolingSet.SetpointHeating, Heating.u) annotation (Line(
          points={{-63,28.8},{-58,28.8},{-58,54},{-52,54}}, color={0,0,127}));
    connect(heatingCoolingSet.SetpointCooling, Cooling.u) annotation (Line(
          points={{-63,25.6},{-58,25.6},{-58,0},{-52,0}}, color={0,0,127}));
    connect(Heating.y, Neutral.u1) annotation (Line(points={{-29,54},{-24,54},{
            -24,30},{-20,30}}, color={255,0,255}));
    connect(Cooling.y, Neutral.u2) annotation (Line(points={{-29,0},{-24,0},{
            -24,22},{-20,22}}, color={255,0,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end HeatingCoolingMode;

  model HeatingCoolingSet "Model to compute heating or cooling mode"
    Modelica.Blocks.Interfaces.RealOutput SetpointHeating
      "1=Heating, 0=No Heating"
      annotation (Placement(transformation(extent={{100,-2},{120,18}})));
    IDEAS.Controls.ControlHeating.RunningMeanTemperatureEN15251
      runningMeanTemperatureEN15251_1
      annotation (Placement(transformation(extent={{-40,0},{-26,16}})));
    Modelica.Blocks.Logical.LessThreshold lessThreshold(threshold=16 + 273)
      annotation (Placement(transformation(extent={{8,-2},{28,18}})));
    Modelica.Blocks.Math.BooleanToReal booleanToReal
      annotation (Placement(transformation(extent={{56,-2},{76,18}})));
    Modelica.Blocks.Interfaces.RealOutput SetpointCooling
      "1=Cooling, 0=No Cooling"
      annotation (Placement(transformation(extent={{100,-34},{120,-14}})));
    Modelica.Blocks.Logical.GreaterThreshold greaterThreshold(threshold=18 +
          273) annotation (Placement(transformation(extent={{8,-34},{28,-14}})));
    Modelica.Blocks.Math.BooleanToReal booleanToReal1
      annotation (Placement(transformation(extent={{56,-34},{76,-14}})));
  equation
    connect(runningMeanTemperatureEN15251_1.TRm, lessThreshold.u) annotation (
        Line(points={{-25.58,8},{6,8}},               color={0,0,127}));
    connect(lessThreshold.y, booleanToReal.u)
      annotation (Line(points={{29,8},{54,8}}, color={255,0,255}));
    connect(booleanToReal.y, SetpointHeating)
      annotation (Line(points={{77,8},{110,8}}, color={0,0,127}));
    connect(greaterThreshold.u, lessThreshold.u) annotation (Line(points={{6,
            -24},{-14,-24},{-14,8},{6,8}}, color={0,0,127}));
    connect(greaterThreshold.y, booleanToReal1.u)
      annotation (Line(points={{29,-24},{54,-24}}, color={255,0,255}));
    connect(booleanToReal1.y, SetpointCooling)
      annotation (Line(points={{77,-24},{110,-24}}, color={0,0,127}));
    connect(SetpointCooling, SetpointCooling)
      annotation (Line(points={{110,-24},{110,-24}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
            extent={{-80,66},{100,-70}},
            lineColor={28,108,200},
            lineThickness=1), Text(
            extent={{-58,36},{82,-32}},
            lineColor={28,108,200},
            textString="Heating/Cooling")}),                       Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end HeatingCoolingSet;

  model Test
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan1(
      addPowerToMedium=false,
      tau=60,
      constantMassFlowRate=1,
      inputType=IDEAS.Fluid.Types.InputType.Continuous,
      m_flow_nominal=6,
      redeclare package Medium = IDEAS.Media.Water)
      annotation (Placement(transformation(extent={{-8,-8},{8,8}},
          rotation=270,
          origin={64,-6})));
    IDEAS.Fluid.MixingVolumes.MixingVolume vol(
      nPorts=2,
      V=2,
      redeclare package Medium = IDEAS.Media.Water,
      m_flow_nominal=6)
                annotation (Placement(transformation(
          extent={{-8,-7},{8,7}},
          rotation=270,
          origin={76,-31})));
    IDEAS.Fluid.Sources.Boundary_pT bou1(
      nPorts=1,
      redeclare package Medium = IDEAS.Media.Water,
      p=150000,
      use_T_in=true,
      T=303.15)
      annotation (Placement(transformation(extent={{-6,-6},{6,6}},
          rotation=270,
          origin={64,22})));
    IDEAS.Fluid.Sensors.TemperatureTwoPort senTem(m_flow_nominal=6, redeclare
        package Medium = IDEAS.Media.Water)
      annotation (Placement(transformation(extent={{-10,-9},{10,9}},
          rotation=90,
          origin={-12,-17})));
    IDEAS.Controls.SetPoints.Table tab1(constantExtrapolation=true, table=[-8,
          30; 15,22])
      annotation (Placement(transformation(extent={{-62,-86},{-42,-66}})));
    IDEAS.Controls.Continuous.LimPID conPID(
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      yMax=1,
      yMin=0,
      initType=Modelica.Blocks.Types.InitPID.NoInit,
      reverseAction=false,
      k=1,
      Ti=10) annotation (Placement(transformation(extent={{-22,-86},{-2,-66}})));
    UpscaleCase900 upscaleCase900_1
      annotation (Placement(transformation(extent={{-50,18},{-20,38}})));
  equation
    connect(vol.ports[1], fan1.port_b) annotation (Line(points={{69,-29.4},{69,
            -22.5},{64,-22.5},{64,-14}}, color={0,127,255}));
    connect(bou1.ports[1], fan1.port_a)
      annotation (Line(points={{64,16},{64,2}}, color={0,127,255}));
    connect(tab1.y, conPID.u_s)
      annotation (Line(points={{-41,-76},{-24,-76}}, color={0,0,127}));
    connect(senTem.T, conPID.u_m) annotation (Line(points={{-21.9,-17},{-44,-17},
            {-44,-50},{-106,-50},{-106,-94},{-12,-94},{-12,-88}}, color={0,0,
            127}));
    connect(conPID.y, fan1.m_flow_in) annotation (Line(points={{-1,-76},{46,-76},
            {46,-54},{92,-54},{92,-6},{73.6,-6}}, color={0,0,127}));
    connect(vol.ports[2], senTem.port_a) annotation (Line(points={{69,-32.6},{
            -12,-32.6},{-12,-27}}, color={0,127,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end Test;

  model RunningMeanTemperature6h
    "Calculate the running mean temperature of 6 hours"

     discrete Modelica.Blocks.Interfaces.RealOutput TRm(unit="K",displayUnit = "degC")
      "Running mean average temperature"
       annotation (Placement(transformation(extent={{96,-10},{116,10}})));
    Modelica.Blocks.Sources.RealExpression TAmb(y=sim.Te)
      annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
    outer IDEAS.BoundaryConditions.SimInfoManager
                                            sim
      annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
  protected
    parameter Modelica.SIunits.Time t_start(fixed=false) "Start time of the model";
    parameter Real coeTRm[6] = {1, 0.8, 0.6, 0.5, 0.4, 0.3}./3.6
      "weighTAmb.yg coefficient for the running average";
    discrete Real[6] TAveHour(each unit="K",each displayUnit = "degC")
      "Vector with the average hour temperatures of the previous nTermRm hours";
    Real intTAmb "integral of TAmb.y";


  initial equation
    intTAmb=0;
    t_start = time;
    TAveHour=ones(6).*sim.Te;
    TRm=sim.Te;
  equation
    der(intTAmb) =  TAmb.y;
  algorithm
    when sample(t_start+3600,3600) then
      // Update of TAveHour
      for i in 2:6 loop
        TAveHour[i] := pre(TAveHour[i-1]);
      end for;
      TAveHour[1] := intTAmb /3600;
      TRm :=TAveHour*coeTRm;
    end when;

  equation
      // reinitialisation of the intTAmb
    when sample(t_start+3600,3600) then
      reinit(intTAmb,0);
    end when;
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics),
      experiment(StopTime=864000),
      __Dymola_experimentSetupOutput,
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
          graphics={
          Rectangle(
            extent={{100,100},{-100,-100}},
            lineColor={100,100,100},
            fillPattern=FillPattern.Solid,
            fillColor={255,255,255}),
          Line(
            points={{0,100},{98,0},{0,-100}},
            color={100,100,100},
            smooth=Smooth.None),
          Text(
            extent={{-100,140},{100,100}},
            lineColor={0,0,255},
            textString="%name"),
          Text(
            extent={{-48,32},{58,-26}},
            lineColor={0,0,255},
            textString="6 hours")}),
  Documentation(revisions="<html>
<ul>
<li>
April 17, 2018, by Damien Picard:<br/>
Add t_start in sample to compute correctly for non zero initial time.<br/>
Use sim.Te as initialization instead of an arbitrary value of 283.15K.
</li>
<li>
January 19, 2015, by Damien Picard:<br/>
First implementation.
</li>
</ul>
</html>"));
  end RunningMeanTemperature6h;

  model Average6h
    "Calculate the running mean temperature of 6 hours"

     discrete Modelica.Blocks.Interfaces.RealOutput TRm(unit="K",displayUnit = "degC")
      "Running mean average temperature"
       annotation (Placement(transformation(extent={{96,-10},{116,10}})));
      Modelica.Blocks.Sources.RealExpression TAmb(y=Input_average)
      annotation (Placement(transformation(extent={{-74,-10},{-54,10}})));
      Modelica.Blocks.Interfaces.RealInput Input_average
      "Connector of setpoint input signal"
      annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  protected
    parameter Modelica.SIunits.Time t_start(fixed=false) "Start time of the model";
    parameter Real coeTRm[6] = {1, 0.8, 0.6, 0.5, 0.4, 0.3}./3.6
      "weighTAmb.yg coefficient for the running average";
    discrete Real[6] TAveHour(each unit="K",each displayUnit = "degC")
      "Vector with the average hour temperatures of the previous nTermRm hours";
    Real intTAmb "integral of TAmb.y";


  initial equation
    intTAmb=0;
    t_start = time;
    TAveHour=ones(6).*Input_average;
    TRm=Input_average;
  equation
    der(intTAmb) =  TAmb.y;
  algorithm
    when sample(t_start+3600,3600) then
      // Update of TAveHour
      for i in 2:6 loop
        TAveHour[i] := pre(TAveHour[i-1]);
      end for;
      TAveHour[1] := intTAmb /3600;
      TRm :=TAveHour*coeTRm;
    end when;

  equation
      // reinitialisation of the intTAmb
    when sample(t_start+3600,3600) then
      reinit(intTAmb,0);
    end when;
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}})),
      experiment(StopTime=864000),
      __Dymola_experimentSetupOutput,
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
          graphics={
          Rectangle(
            extent={{100,100},{-100,-100}},
            lineColor={100,100,100},
            fillPattern=FillPattern.Solid,
            fillColor={255,255,255}),
          Line(
            points={{0,100},{98,0},{0,-100}},
            color={100,100,100},
            smooth=Smooth.None),
          Text(
            extent={{-100,140},{100,100}},
            lineColor={0,0,255},
            textString="%name"),
          Text(
            extent={{-48,32},{58,-26}},
            lineColor={0,0,255},
            textString="6 hours")}),
  Documentation(revisions="<html>
<ul>
<li>
April 17, 2018, by Damien Picard:<br/>
Add t_start in sample to compute correctly for non zero initial time.<br/>
Use Input_average as initialization instead of an arbitrary value of 283.15K.
</li>
<li>
January 19, 2015, by Damien Picard:<br/>
First implementation.
</li>
</ul>
</html>"));
  end Average6h;

  model RBC_ventilation

    extends Thesis_Bram_Stockman.RBC(BufferTank(nPorts=7));
     package MediumAir = IDEAS.Media.Air;


    IDEAS.Fluid.HeatExchangers.ConstantEffectiveness hex1(
      eps=0.8,
      dp1_nominal=0,
      dp2_nominal=0,
      redeclare package Medium2 = MediumAir,
      m1_flow_nominal=0.72,
      m2_flow_nominal=0.72,
      redeclare package Medium1 = MediumAir)             annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-122,-138})));
    IDEAS.Fluid.Sources.Boundary_pT
                        bouAir(
      redeclare package Medium = MediumAir,
      use_T_in=true,
      nPorts=2)      "Air boundary with constant temperature"
      annotation (Placement(transformation(
          extent={{-10,10},{10,-10}},
          origin={-170,-142})));
    Modelica.Blocks.Sources.RealExpression realExpression(y=sim.Te)
      annotation (Placement(transformation(extent={{-240,-106},{-220,-86}})));
    IDEAS.Fluid.HeatExchangers.ConstantEffectiveness heating_coil(
      eps=0.8,
      dp1_nominal=0,
      dp2_nominal=0,
      redeclare package Medium2 = MediumAir,
      m1_flow_nominal=0.72,
      m2_flow_nominal=0.72,
      redeclare package Medium1 = Medium) annotation (Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=180,
          origin={-56,-98})));
    IDEAS.Fluid.HeatExchangers.ConstantEffectiveness cooling_coil(
      eps=0.8,
      dp1_nominal=0,
      dp2_nominal=0,
      redeclare package Medium2 = MediumAir,
      m1_flow_nominal=0.72,
      m2_flow_nominal=0.72,
      redeclare package Medium1 = Medium) annotation (Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=180,
          origin={-84,-96})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan4(
      addPowerToMedium=false,
      tau=60,
      constantMassFlowRate=1,
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      use_inputFilter=false,
      m_flow_nominal=0.72,
      inputType=IDEAS.Fluid.Types.InputType.Stages,
      redeclare package Medium = MediumAir,
      T_start=297.15)
      annotation (Placement(transformation(extent={{-8,-8},{8,8}},
          rotation=270,
          origin={-28,-98})));
    IDEAS.Fluid.Actuators.Dampers.VAVBoxExponential
                                        vavDam(
      redeclare package Medium = MediumAir,
      from_dp=true,
      m_flow_nominal=0.72,
      dp_nominal=1)                     "Damper" annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          origin={-26,-130},
          rotation=270)));
    IDEAS.Controls.SetPoints.OccupancySchedule occSch(occupancy=3600*{7,19,31,
          43,55,67,79,91,103,115,127,139}, period=7*24*3600)
      annotation (Placement(transformation(extent={{-72,-242},{-52,-222}})));
    Modelica.Blocks.Math.BooleanToInteger booleanToInteger
      annotation (Placement(transformation(extent={{-42,-248},{-22,-228}})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan5(
      redeclare IDEAS.Fluid.Movers.Data.Generic per,
      redeclare package Medium = Medium,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      addPowerToMedium=false,
      tau=60,
      use_inputFilter=false,
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      inputType=IDEAS.Fluid.Types.InputType.Stages,
      m_flow_nominal=3)
      annotation (Placement(transformation(extent={{-9,-8},{9,8}},
          rotation=90,
          origin={-124,-51})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan6(
      redeclare IDEAS.Fluid.Movers.Data.Generic per,
      redeclare package Medium = Medium,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      addPowerToMedium=false,
      tau=60,
      use_inputFilter=false,
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      inputType=IDEAS.Fluid.Types.InputType.Stages,
      m_flow_nominal=3)
      annotation (Placement(transformation(extent={{-9,-8},{9,8}},
          rotation=180,
          origin={-12,-63})));
    Modelica.Blocks.Math.BooleanToInteger booleanToInteger1
      annotation (Placement(transformation(extent={{-168,-42},{-148,-22}})));
    Modelica.Blocks.Math.BooleanToInteger booleanToInteger2
      annotation (Placement(transformation(extent={{-40,-278},{-20,-258}})));
    IDEAS.Controls.Continuous.LimPID
                               conDam(controllerType=Modelica.Blocks.Types.SimpleController.P,
        yMin=0.1) "Controller for damper"
      annotation (Placement(transformation(extent={{14,-250},{34,-230}})));
    Modelica.Blocks.Sources.Constant const(k=273 + 21)
      annotation (Placement(transformation(extent={{62,-274},{42,-254}})));
  equation
    connect(bouAir.T_in, realExpression.y)
      annotation (Line(points={{-182,-146},{-182,-96},{-219,-96}},
                                                       color={0,0,127}));
    connect(bouAir.ports[1], hex1.port_b1) annotation (Line(points={{-160,-144},
            {-132,-144}},                      color={0,127,255}));
    connect(hex1.port_a1, upscaleCase900_HVAC.port_b[1]) annotation (Line(
          points={{-112,-144},{-50,-144},{-50,-162},{-45,-162}},
                                                               color={0,127,255}));
    connect(bouAir.ports[2], hex1.port_a2)
      annotation (Line(points={{-160,-140},{-140,-140},{-140,-132},{-132,-132}},
                                                       color={0,127,255}));
    connect(hex1.port_b2, cooling_coil.port_a2)
      annotation (Line(points={{-112,-132},{-106,-132},{-106,-102},{-94,-102}},
                                                       color={0,127,255}));
    connect(cooling_coil.port_b2, heating_coil.port_a2)
      annotation (Line(points={{-74,-102},{-70,-102},{-70,-104},{-66,-104}},
                                                     color={0,127,255}));
    connect(heating_coil.port_b2, fan4.port_a) annotation (Line(points={{-46,
            -104},{-46,-90},{-28,-90}},color={0,127,255}));
    connect(fan4.port_b, vavDam.port_a)
      annotation (Line(points={{-28,-106},{-28,-120},{-26,-120}},
                                                       color={0,127,255}));
    connect(vavDam.port_b, upscaleCase900_HVAC.port_a[1]) annotation (Line(
          points={{-26,-140},{-42,-140},{-42,-162},{-41,-162}}, color={0,127,
            255}));
    connect(occSch.occupied, booleanToInteger.u)
      annotation (Line(points={{-51,-238},{-44,-238}}, color={255,0,255}));
    connect(booleanToInteger.y, fan4.stage) annotation (Line(points={{-21,-238},
            {-4,-238},{-4,-98},{-18.4,-98}}, color={255,127,0}));
    connect(cooling_coil.port_a1, hex.port_b2) annotation (Line(points={{-74,-90},
            {-74,-50},{30,-50},{30,24},{14,24},{14,18}},      color={0,127,255}));
    connect(cooling_coil.port_b1, fan5.port_a) annotation (Line(points={{-94,-90},
            {-112,-90},{-112,-60},{-124,-60}},      color={0,127,255}));
    connect(fan5.port_b, hex.port_a2) annotation (Line(points={{-124,-42},{-124,
            -38},{8,-38},{8,-2},{14,-2}}, color={0,127,255}));
    connect(BufferTank.ports[6], heating_coil.port_b1) annotation (Line(points=
            {{43,56},{42,56},{42,-48},{-66,-48},{-66,-92}}, color={0,127,255}));
    connect(heating_coil.port_a1, fan6.port_b) annotation (Line(points={{-46,-92},
            {-30,-92},{-30,-63},{-21,-63}},      color={0,127,255}));
    connect(BufferTank.ports[7], fan6.port_a) annotation (Line(points={{43,56},
            {50,56},{50,-54},{-3,-54},{-3,-63}}, color={0,127,255}));
    connect(fan5.stage, booleanToInteger1.y) annotation (Line(points={{-133.6,
            -51},{-140,-51},{-140,-32},{-147,-32}}, color={255,127,0}));
    connect(booleanToInteger1.u, not1.y) annotation (Line(points={{-170,-32},{
            -174,-32},{-174,120},{-179.4,120}}, color={255,0,255}));
    connect(booleanToInteger2.u, not1.u) annotation (Line(points={{-42,-268},{
            -200,-268},{-200,120},{-193.2,120}}, color={255,0,255}));
    connect(booleanToInteger2.y, fan6.stage) annotation (Line(points={{-19,-268},
            {0,-268},{0,-72.6},{-12,-72.6}}, color={255,127,0}));
    connect(conDam.u_s, average6h.Input_average) annotation (Line(points={{12,-240},
            {12,-178},{70,-178}},       color={0,0,127}));
    connect(const.y, conDam.u_m) annotation (Line(points={{41,-264},{32,-264},{
            32,-252},{24,-252}}, color={0,0,127}));
    connect(conDam.y, vavDam.y) annotation (Line(points={{35,-240},{48,-240},{
            48,-206},{-14,-206},{-14,-130}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end RBC_ventilation;

  model PI_fan_tuning

    package Medium = IDEAS.Media.Water;

    IDEAS.Fluid.HeatExchangers.RadiantSlab.EmbeddedPipe embeddedPipe(
      redeclare package Medium = Medium,
      m_flow_nominal=6,
      A_floor=1200,
      redeclare IDEAS.Fluid.HeatExchangers.RadiantSlab.BaseClasses.FH_Standard1
        RadSlaCha,
      T_start=297.15)
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={46,-70})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan1(
      redeclare package Medium = Medium,
      addPowerToMedium=false,
      tau=60,
      constantMassFlowRate=1,
      m_flow_nominal=6,
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      use_inputFilter=false,
      inputType=IDEAS.Fluid.Types.InputType.Constant,
      T_start=297.15)
      annotation (Placement(transformation(extent={{-8,-8},{8,8}},
          rotation=270,
          origin={76,-52})));
    IDEAS.Fluid.MixingVolumes.MixingVolume vol(
      redeclare package Medium = Medium,
      nPorts=5,
      m_flow_nominal=fan3.m_flow_nominal,
      V=1,
      T_start=308.15)
                annotation (Placement(transformation(
          extent={{-9,-8},{9,8}},
          rotation=0,
          origin={71,94})));
    IDEAS.Fluid.Actuators.Valves.Simplified.ThreeWayValveSwitch
                                                threeWayValveSwitch1(
      redeclare package Medium = Medium,
      m_flow_nominal=1,
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      T_start=297.15)                               annotation (Placement(
          transformation(
          extent={{-8,-8},{8,8}},
          rotation=270,
          origin={76,18})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan2(
      redeclare package Medium = Medium,
      addPowerToMedium=false,
      tau=60,
      use_inputFilter=false,
      m_flow_nominal=6,
      inputType=IDEAS.Fluid.Types.InputType.Constant,
      constantMassFlowRate=6)
      annotation (Placement(transformation(extent={{-8,-8},{8,8}},
          rotation=0,
          origin={46,58})));
    IDEAS.Fluid.HeatPumps.ScrollWaterToWater  heaPum(
      redeclare package Medium1 = Medium,
      redeclare package Medium2 = Medium,
      allowFlowReversal1=false,
      allowFlowReversal2=false,
      redeclare package ref = IDEAS.Media.Refrigerants.R410A,
      datHeaPum=
          IDEAS.Fluid.HeatPumps.Data.ScrollWaterToWater.Heating.ClimateMaster_TMW036_12kW_4_90COP_R410A(),
      m1_flow_nominal=fan2.m_flow_nominal,
      m2_flow_nominal=fan3.m_flow_nominal,
      scaling_factor=2,
      dp1_nominal=0,
      dp2_nominal=0,
      enable_variable_speed=false,
      T1_start=308.15,
      T2_start=283.15)
      annotation (Placement(transformation(extent={{10,-10},{-10,10}},
          rotation=270,
          origin={-36,48})));
    IDEAS.Fluid.Geothermal.Borefields.OneUTube borFie(redeclare package Medium =
          Medium, borFieDat=
          IDEAS.Fluid.Geothermal.Borefields.Data.Borefield.Example(
          filDat=IDEAS.Fluid.Geothermal.Borefields.Data.Filling.Bentonite(
            kFil=0.6,
            cFil=1650,
            dFil=1000,
            steadyState=true),
          soiDat=IDEAS.Fluid.Geothermal.Borefields.Data.Soil.SandStone(
            kSoi=2.2,
            cSoi=2470,
            dSoi=1000),
          conDat=IDEAS.Fluid.Geothermal.Borefields.Data.Configuration.Example(
            borCon=IDEAS.Fluid.Geothermal.Borefields.Types.BoreholeConfiguration.SingleUTube,
            use_Rb=true,
            Rb=0.266,
            mBor_flow_nominal=6.5/18,
            mBorFie_flow_nominal=6.5,
            hBor=100,
            dBor=1,
            nBor=18,
            cooBor={{0,0},{0,6},{6,0},{6,6},{0,12},{6,12},{12,12},{12,6},{12,0},{0,
              18},{6,18},{12,18},{18,18},{18,12},{18,6},{18,0},{0,24},{6,24}},
            kTub=0.38)))
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-66,-28})));
    IDEAS.Fluid.HeatExchangers.ConstantEffectiveness hex(
      redeclare package Medium1 = Medium,
      redeclare package Medium2 = Medium,
      eps=0.8,
      m1_flow_nominal=fan3.m_flow_nominal,
      m2_flow_nominal=fan3.m_flow_nominal,
      dp1_nominal=0,
      dp2_nominal=0)                                     annotation (Placement(
          transformation(
          extent={{-10,10},{10,-10}},
          rotation=270,
          origin={8,8})));
    IDEAS.Fluid.Actuators.Valves.Simplified.ThreeWayValveSwitch
                                                threeWayValveSwitch(
      redeclare package Medium = Medium,
      m_flow_nominal=fan3.m_flow_nominal,
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
                                                    annotation (Placement(
          transformation(
          extent={{8,-8},{-8,8}},
          rotation=90,
          origin={-88,18})));
    IDEAS.Fluid.Sources.Boundary_pT bou(
      redeclare package Medium = Medium,
      nPorts=1,
      p=150000)
      annotation (Placement(transformation(extent={{-120,52},{-106,66}})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan3(
      redeclare IDEAS.Fluid.Movers.Data.Generic per,
      redeclare package Medium = Medium,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      addPowerToMedium=false,
      tau=60,
      use_inputFilter=false,
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      m_flow_nominal=6,
      inputType=IDEAS.Fluid.Types.InputType.Constant)
      annotation (Placement(transformation(extent={{-9,-8},{9,8}},
          rotation=90,
          origin={-88,-5})));
    IDEAS.Fluid.Sources.Boundary_pT bou1(
      redeclare package Medium = Medium,
      p=150000,
      nPorts=1)
      annotation (Placement(transformation(extent={{-7,-7},{7,7}},
          rotation=270,
          origin={105,77})));
    IDEAS.Fluid.Sensors.TemperatureTwoPort senTem(
      m_flow_nominal=6,
      redeclare package Medium = Medium,
      tau=0,
      initType=Modelica.Blocks.Types.Init.SteadyState,
      T_start=297.15)
      annotation (Placement(transformation(extent={{74,-64},{60,-76}})));
    IDEAS.Fluid.Sensors.TemperatureTwoPort senTem1(
      m_flow_nominal=6,
      redeclare package Medium = Medium,
      tau=0,
      initType=Modelica.Blocks.Types.Init.SteadyState,
      T_start=297.15)
      annotation (Placement(transformation(extent={{34,-64},{20,-76}})));
    IDEAS.Fluid.Actuators.Valves.Simplified.ThreeWayValveMotor
                                                threeWayValveMotor(
      redeclare package Medium = Medium,
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      m_flow_nominal=6,
      allowFlowReversal=true,
      T_start=297.15)                               annotation (Placement(
          transformation(
          extent={{-8,8},{8,-8}},
          rotation=270,
          origin={20,-16})));
    IDEAS.Fluid.FixedResistances.Junction jun(
      redeclare package Medium = Medium,
      m_flow_nominal={6,6,6},
      dp_nominal={0,0,0},
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      T_start=303.15)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=270,
          origin={76,-16})));
    IDEAS.Fluid.FixedResistances.Junction jun1(
      redeclare package Medium = Medium,
      m_flow_nominal={6,6,6},
      dp_nominal={0,0,0},
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-40,-28})));
    IDEAS.Fluid.MixingVolumes.MixingVolume vol1(
      V=1200*2.8,
      m_flow_nominal=0,
      redeclare package Medium = Medium,
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      nPorts=1,
      p_start=100000,
      T_start=294.15)   annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={4,-96})));
    Modelica.Blocks.Sources.BooleanConstant booleanConstant(k=false)
      annotation (Placement(transformation(extent={{-140,8},{-120,28}})));
    Modelica.Blocks.Sources.BooleanConstant booleanConstant1(k=false)
      annotation (Placement(transformation(extent={{120,8},{100,28}})));
    Modelica.Blocks.Sources.Step step(
      offset=0,
      startTime=200,
      height=6)
      annotation (Placement(transformation(extent={{-64,-80},{-44,-60}})));
    Modelica.Blocks.Sources.IntegerConstant integerConstant(k=1)
      annotation (Placement(transformation(extent={{-80,84},{-60,104}})));
    IDEAS.Fluid.Sources.Boundary_pT bou2(
      redeclare package Medium = Medium,
      nPorts=1,
      p=100000)
      annotation (Placement(transformation(extent={{-7,-7},{7,7}},
          rotation=270,
          origin={-37,-111})));
    IDEAS.Fluid.Sensors.TemperatureTwoPort senTem2(
      redeclare package Medium = Medium,
      m_flow_nominal=6,
      T_start=283.15)
      annotation (Placement(transformation(extent={{-56,48},{-76,68}})));
    IDEAS.Fluid.Sensors.TemperatureTwoPort senTem3(
      redeclare package Medium = Medium,
      m_flow_nominal=1,
      T_start=308.15)
      annotation (Placement(transformation(extent={{22,48},{2,68}})));
  equation
    connect(fan2.port_b,vol. ports[2])
      annotation (Line(points={{54,58},{69.56,58},{69.56,86}},
                                                         color={0,127,255}));
    connect(heaPum.port_a1,vol. ports[3])
      annotation (Line(points={{-30,38},{71,38},{71,86}}, color={0,127,255}));
    connect(fan3.port_a,borFie. port_b) annotation (Line(points={{-88,-14},{-88,
            -28},{-76,-28}}, color={0,127,255}));
    connect(threeWayValveSwitch.port_b,fan3. port_b)
      annotation (Line(points={{-88,10},{-88,4}}, color={0,127,255}));
    connect(threeWayValveSwitch.port_a2,hex. port_a1)
      annotation (Line(points={{-80,18},{2,18}}, color={0,127,255}));
    connect(threeWayValveSwitch1.port_a2,hex. port_b2)
      annotation (Line(points={{68,18},{14,18}}, color={0,127,255}));
    connect(embeddedPipe.port_a,senTem. port_b)
      annotation (Line(points={{56,-70},{60,-70}}, color={0,127,255}));
    connect(fan1.port_b,senTem. port_a) annotation (Line(points={{76,-60},{76,
            -70},{74,-70}}, color={0,127,255}));
    connect(embeddedPipe.port_b,senTem1. port_a)
      annotation (Line(points={{36,-70},{34,-70}}, color={0,127,255}));
    connect(threeWayValveMotor.port_b,senTem1. port_b)
      annotation (Line(points={{20,-24},{20,-70}}, color={0,127,255}));
    connect(threeWayValveMotor.port_a1,hex. port_a2) annotation (Line(points={{20,-8},
            {18,-8},{18,-2},{14,-2}},          color={0,127,255}));
    connect(vol.ports[4],threeWayValveSwitch1. port_a1) annotation (Line(points={{72.44,
            86},{76,86},{76,26}},        color={0,127,255}));
    connect(threeWayValveMotor.port_a2, jun.port_3)
      annotation (Line(points={{28,-16},{66,-16}}, color={0,127,255}));
    connect(threeWayValveSwitch1.port_b, jun.port_1)
      annotation (Line(points={{76,10},{76,-6}}, color={0,127,255}));
    connect(fan1.port_a, jun.port_2)
      annotation (Line(points={{76,-44},{76,-26}}, color={0,127,255}));
    connect(heaPum.port_b2, jun1.port_3)
      annotation (Line(points={{-42,38},{-42,10},{-40,10},{-40,-18}},
                                                             color={0,127,255}));
    connect(borFie.port_a, jun1.port_2)
      annotation (Line(points={{-56,-28},{-50,-28}}, color={0,127,255}));
    connect(hex.port_b1, jun1.port_1) annotation (Line(points={{2,-2},{-8,-2},{
            -8,-28},{-30,-28}},
                             color={0,127,255}));
    connect(embeddedPipe.heatPortEmb[1], vol1.heatPort)
      annotation (Line(points={{46,-80},{46,-96},{14,-96}}, color={191,0,0}));
    connect(threeWayValveSwitch.switch, booleanConstant.y)
      annotation (Line(points={{-94.4,18},{-119,18}}, color={255,0,255}));
    connect(threeWayValveSwitch1.switch, booleanConstant1.y)
      annotation (Line(points={{82.4,18},{99,18}},  color={255,0,255}));
    connect(threeWayValveMotor.ctrl, step.y) annotation (Line(points={{11.36,
            -16},{0,-16},{0,-70},{-43,-70}},
                                        color={0,0,127}));
    connect(heaPum.stage, integerConstant.y) annotation (Line(points={{-33,36},
            {-32,36},{-32,24},{-50,24},{-50,94},{-59,94}}, color={255,127,0}));
    connect(bou2.ports[1], vol1.ports[1]) annotation (Line(points={{-37,-118},{
            -18,-118},{-18,-86},{4,-86}}, color={0,127,255}));
    connect(threeWayValveSwitch.port_a1, senTem2.port_b) annotation (Line(
          points={{-88,26},{-88,58},{-76,58}}, color={0,127,255}));
    connect(bou.ports[1], senTem2.port_b) annotation (Line(points={{-106,59},{
            -97,59},{-97,58},{-76,58}}, color={0,127,255}));
    connect(heaPum.port_a2, senTem2.port_a)
      annotation (Line(points={{-42,58},{-56,58}}, color={0,127,255}));
    connect(hex.port_b2, vol.ports[5]) annotation (Line(points={{14,18},{16,18},
            {16,32},{73.88,32},{73.88,86}}, color={0,127,255}));
    connect(bou1.ports[1], threeWayValveSwitch1.port_a1) annotation (Line(
          points={{105,70},{90,70},{90,60},{76,60},{76,26}}, color={0,127,255}));
    connect(heaPum.port_b1, senTem3.port_b)
      annotation (Line(points={{-30,58},{2,58}}, color={0,127,255}));
    connect(senTem3.port_a, fan2.port_a)
      annotation (Line(points={{22,58},{38,58}}, color={0,127,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-260,-220},
              {260,140}})), Diagram(coordinateSystem(preserveAspectRatio=false,
            extent={{-260,-220},{260,140}})));
  end PI_fan_tuning;

  model PIDcontrol
    "Example model for the heater with prescribed outlet temperature and air as the medium"
    extends Modelica.Icons.Example;
    extends IDEAS.Fluid.HeatExchangers.Examples.BaseClasses.Heater(
      redeclare package Medium = IDEAS.Media.Air,
      m_flow_nominal=V*1.2*6/3600,
      Q_flow_nominal=30*6*6,
      mov(dp_nominal=1200, nominalValuesDefineDefaultPressureCurve=true),
      TOut(y=273.15 + 16));

    IDEAS.Controls.SetPoints.Table
                             tab(table=[0,273.15 + 15; 1,273.15 + 30])
      "Temperature set point"
      annotation (Placement(transformation(extent={{-32,20},{-12,40}})));
    IDEAS.Fluid.HeatExchangers.Heater_T
             hea(
      redeclare package Medium = Medium,
      m_flow_nominal=m_flow_nominal,
      dp_nominal=1000,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      QMax_flow=Q_flow_nominal) "Heater"
      annotation (Placement(transformation(extent={{-8,-50},{12,-30}})));
  equation
    connect(conPI.y, tab.u)
      annotation (Line(points={{-39,30},{-34,30}}, color={0,0,127}));
    connect(mov.port_b, hea.port_a)
      annotation (Line(points={{-50,-40},{-8,-40}}, color={0,127,255}));
    connect(THeaOut.port_a, hea.port_b)
      annotation (Line(points={{20,-40},{12,-40}}, color={0,127,255}));
    connect(tab.y, hea.TSet)
      annotation (Line(points={{-11,30},{-10,30},{-10,-32}}, color={0,0,127}));
    annotation ( Documentation(info="<html>
<p>
This example illustrates how to use the heater model that takes as an
input the leaving fluid temperature.
</p>
<p>
The model consist of an air volume with heat loss to the ambient.
The set point of the air temperature is different between night and day.
The heater tracks the set point temperature, except for the periods in
which the air temperature is above the set point.
</p>
<p>
See
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.Examples.AirHeater_u\">
IDEAS.Fluid.HeatExchangers.Examples.AirHeater_u</a>
for a model that takes the heating power as an input.
</p>
</html>",   revisions="<html>
<ul>
<li>
May 8, 2017, by Michael Wetter:<br/>
Updated heater model.<br/>
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/763\">
IDEAS, #763</a>.
</li>
<li>
January 6, 2015, by Michael Wetter:<br/>
Revised implementation.
</li>
<li>
November 12, 2014, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"),
      __Dymola_Commands(file= "modelica://IDEAS/Resources/Scripts/Dymola/Fluid/HeatExchangers/Examples/AirHeater_T.mos"
          "Simulate and plot"),
      experiment(
        StopTime=172800,
        Tolerance=1e-08),
      Diagram(coordinateSystem(extent={{-100,-100},{120,100}})),
      Icon(coordinateSystem(extent={{-100,-100},{120,100}})));
  end PIDcontrol;

  model HoldTest "Model to hold a value of a function at a given time"
    HoldPermanent holdPermanent(startTime=0.5)
      annotation (Placement(transformation(extent={{-28,12},{-8,32}})));
    Modelica.Blocks.Sources.Cosine cosine(amplitude=1, freqHz=1)
      annotation (Placement(transformation(extent={{-68,12},{-48,32}})));
  equation
    connect(holdPermanent.u, cosine.y)
      annotation (Line(points={{-30,22},{-47,22}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end HoldTest;

  block HoldPermanent "Hold value of a function permanent"
    extends Modelica.Blocks.Interfaces.DiscreteSISO;

  equation
    when sampleTrigger and not
                              (time > startTime) then
      y = u;
    end when;

  end HoldPermanent;

  model SimInfoManagar_constant "Keeps wheater data constant"
    extends IDEAS.BoundaryConditions.SimInfoManager;
    HoldPermanent HDirTil(startTime=43200)
      annotation (Placement(transformation(extent={{134,88},{140,94}})));
    HoldPermanent HSkyDifTil
      annotation (Placement(transformation(extent={{134,78},{140,84}})));
    HoldPermanent HGroDifTil
      annotation (Placement(transformation(extent={{134,68},{140,74}})));
    HoldPermanent angInc
      annotation (Placement(transformation(extent={{134,58},{140,64}})));
    HoldPermanent angZenHold
      annotation (Placement(transformation(extent={{134,48},{140,54}})));
    HoldPermanent angAzi
      annotation (Placement(transformation(extent={{134,38},{140,44}})));
    HoldPermanent Tenv
      annotation (Placement(transformation(extent={{134,28},{140,34}})));
    HoldPermanent TeHold
      annotation (Placement(transformation(extent={{134,18},{140,24}})));
  equation
    connect(HGloHor.y, HGloHor.u)
      annotation (Line(points={{-77.6,112},{-86.8,112}}, color={0,0,127}));
    annotation (Diagram(coordinateSystem(extent={{-100,-100},{180,100}})), Icon(
          coordinateSystem(extent={{-100,-100},{180,100}})));
  end SimInfoManagar_constant;

  model RBC_nightSetBack
    package Medium = IDEAS.Media.Water;
    UpscaleCase900 upscaleCase900_HVAC
      annotation (Placement(transformation(extent={{-62,-112},{-32,-92}})));
    IDEAS.Fluid.HeatExchangers.RadiantSlab.EmbeddedPipe embeddedPipe(
      redeclare package Medium = Medium,
      A_floor=1200,
      redeclare IDEAS.Fluid.HeatExchangers.RadiantSlab.BaseClasses.FH_Standard1
        RadSlaCha,
      m_flow_nominal=6,
      T_start=308.15)
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={46,-70})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan1(
      redeclare package Medium = Medium,
      addPowerToMedium=false,
      tau=60,
      constantMassFlowRate=1,
      inputType=IDEAS.Fluid.Types.InputType.Continuous,
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      use_inputFilter=false,
      T_start=308.15,
      m_flow_nominal=6)
      annotation (Placement(transformation(extent={{-8,-8},{8,8}},
          rotation=270,
          origin={76,-52})));
    IDEAS.Fluid.MixingVolumes.MixingVolume vol(
      redeclare package Medium = Medium,
      nPorts=5,
      m_flow_nominal=fan3.m_flow_nominal,
      V=10,
      T_start=308.15)
                annotation (Placement(transformation(
          extent={{-9,-8},{9,8}},
          rotation=0,
          origin={43,64})));
    IDEAS.Fluid.Actuators.Valves.Simplified.ThreeWayValveSwitch
                                                threeWayValveSwitch1(
      redeclare package Medium = Medium,
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      m_flow_nominal=6,
      T_start=308.15)                               annotation (Placement(
          transformation(
          extent={{-8,-8},{8,8}},
          rotation=270,
          origin={76,18})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan2(
      redeclare package Medium = Medium,
      addPowerToMedium=false,
      tau=60,
      constantMassFlowRate=1,
      inputType=IDEAS.Fluid.Types.InputType.Continuous,
      use_inputFilter=false,
      m_flow_nominal=6,
      T_start=308.15)
      annotation (Placement(transformation(extent={{-8,-8},{8,8}},
          rotation=0,
          origin={6,52})));
    IDEAS.Fluid.HeatPumps.ScrollWaterToWater  heaPum(
      redeclare package Medium1 = Medium,
      redeclare package Medium2 = Medium,
      allowFlowReversal1=false,
      allowFlowReversal2=false,
      redeclare package ref = IDEAS.Media.Refrigerants.R410A,
      datHeaPum=
          IDEAS.Fluid.HeatPumps.Data.ScrollWaterToWater.Heating.ClimateMaster_TMW036_12kW_4_90COP_R410A(),
      m1_flow_nominal=fan2.m_flow_nominal,
      m2_flow_nominal=fan3.m_flow_nominal,
      enable_variable_speed=false,
      dp1_nominal=0,
      dp2_nominal=0,
      scaling_factor=4)
      annotation (Placement(transformation(extent={{10,-10},{-10,10}},
          rotation=270,
          origin={-20,42})));

    IDEAS.Fluid.Geothermal.Borefields.OneUTube borFie(redeclare package Medium =
          Medium, borFieDat=
          IDEAS.Fluid.Geothermal.Borefields.Data.Borefield.Example(
            filDat=IDEAS.Fluid.Geothermal.Borefields.Data.Filling.Bentonite(
              kFil=0.6,
              cFil=1650,
              dFil=1000,
              steadyState=true),
            soiDat=IDEAS.Fluid.Geothermal.Borefields.Data.Soil.SandStone(
              kSoi=2.2,
              cSoi=2470,
              dSoi=1000),
            conDat=IDEAS.Fluid.Geothermal.Borefields.Data.Configuration.Example(
              borCon=IDEAS.Fluid.Geothermal.Borefields.Types.BoreholeConfiguration.SingleUTube,
              use_Rb=true,
              Rb=0.266,
              mBor_flow_nominal=6.5/18,
              mBorFie_flow_nominal=6.5,
              hBor=100,
              dBor=1,
              nBor=18,
              cooBor={{0,0},{0,6},{6,0},{6,6},{0,12},{6,12},{12,12},{12,6},{12,
              0},{0,18},{6,18},{12,18},{18,18},{18,12},{18,6},{18,0},{0,24},{6,
              24}},
              kTub=0.38)))
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-66,-28})));
    IDEAS.Fluid.HeatExchangers.ConstantEffectiveness hex(
      redeclare package Medium1 = Medium,
      redeclare package Medium2 = Medium,
      eps=0.8,
      m1_flow_nominal=fan3.m_flow_nominal,
      dp1_nominal=0,
      dp2_nominal=0,
      m2_flow_nominal=6)                                 annotation (Placement(
          transformation(
          extent={{-10,10},{10,-10}},
          rotation=270,
          origin={8,8})));
    IDEAS.Fluid.Actuators.Valves.Simplified.ThreeWayValveSwitch
                                                threeWayValveSwitch(
      redeclare package Medium = Medium,
      m_flow_nominal=fan3.m_flow_nominal,
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
                                                    annotation (Placement(
          transformation(
          extent={{8,-8},{-8,8}},
          rotation=90,
          origin={-78,18})));
    IDEAS.Fluid.Sources.Boundary_pT bou(
      redeclare package Medium = Medium,
      nPorts=1,
      p=150000)
      annotation (Placement(transformation(extent={{-98,52},{-84,66}})));
    IDEAS.Fluid.Movers.FlowControlled_m_flow fan3(
      redeclare IDEAS.Fluid.Movers.Data.Generic per,
      redeclare package Medium = Medium,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      addPowerToMedium=false,
      tau=60,
      use_inputFilter=false,
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      inputType=IDEAS.Fluid.Types.InputType.Continuous,
      m_flow_nominal=6)
      annotation (Placement(transformation(extent={{-9,-8},{9,8}},
          rotation=90,
          origin={-78,-5})));
    HeatingCoolingSet heatingCoolingSet
      annotation (Placement(transformation(extent={{-250,108},{-228,130}})));
    IDEAS.Fluid.Sources.Boundary_pT bou1(
      redeclare package Medium = Medium,
      nPorts=1,
      p=150000)
      annotation (Placement(transformation(extent={{-7,-7},{7,7}},
          rotation=270,
          origin={77,77})));
    Modelica.Blocks.Logical.Hysteresis
                                     hysteresis(
      pre_y_start=false,
      uLow=273.15 + 32,
      uHigh=273.15 + 37)
             annotation (Placement(transformation(extent={{136,56},{154,74}})));
    Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
      annotation (Placement(transformation(extent={{34,80},{48,94}})));
    Modelica.Blocks.Math.RealToBoolean realToBoolean
      annotation (Placement(transformation(extent={{-218,114},{-206,126}})));
    Modelica.Blocks.Logical.Not not1
      annotation (Placement(transformation(extent={{-192,114},{-180,126}})));
    Modelica.Blocks.Math.BooleanToReal booleanToReal
      annotation (Placement(transformation(extent={{170,20},{184,34}})));
    Modelica.Blocks.Logical.Not not2
      annotation (Placement(transformation(extent={{168,58},{182,72}})));
    Modelica.Blocks.Logical.And and1
      annotation (Placement(transformation(extent={{144,20},{158,34}})));
    Modelica.Blocks.Logical.Not not3
      annotation (Placement(transformation(extent={{106,0},{120,14}})));
    Modelica.Blocks.Math.RealToInteger realToInteger1
      annotation (Placement(transformation(extent={{210,20},{224,34}})));
    Modelica.Blocks.Logical.Or or1
      annotation (Placement(transformation(extent={{-152,-12},{-138,2}})));
    Modelica.Blocks.Math.Gain gain(k=6) annotation (Placement(transformation(
          extent={{-8,-8},{8,8}},
          rotation=270,
          origin={6,82})));
    Modelica.Blocks.Math.BooleanToReal booleanToReal1
      annotation (Placement(transformation(extent={{-132,-12},{-118,2}})));
    Modelica.Blocks.Math.Gain gain1(k=6) annotation (Placement(transformation(
          extent={{-7,-7},{7,7}},
          rotation=0,
          origin={-103,-5})));
    IDEAS.Fluid.Sensors.TemperatureTwoPort senTem(
      m_flow_nominal=6,
      redeclare package Medium = Medium,
      tau=0,
      initType=Modelica.Blocks.Types.Init.SteadyState,
      T_start=308.15)
      annotation (Placement(transformation(extent={{74,-64},{60,-76}})));
    IDEAS.Fluid.Sensors.TemperatureTwoPort senTem1(
      m_flow_nominal=6,
      redeclare package Medium = Medium,
      tau=0,
      initType=Modelica.Blocks.Types.Init.SteadyState,
      T_start=308.15)
      annotation (Placement(transformation(extent={{34,-64},{20,-76}})));
    IDEAS.Fluid.Actuators.Valves.Simplified.ThreeWayValveMotor
                                                threeWayValveMotor(
      redeclare package Medium = Medium,
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      allowFlowReversal=true,
      T_start=308.15,
      m_flow_nominal=6)                             annotation (Placement(
          transformation(
          extent={{-8,8},{8,-8}},
          rotation=270,
          origin={20,-38})));
    IDEAS.Controls.Continuous.LimPID conPID(
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      yMax=1,
      xi_start=0,
      k=1,
      yMin=0,
      Ti=60)
      annotation (Placement(transformation(extent={{156,-116},{172,-100}})));
    IDEAS.Controls.Continuous.LimPID conPID1(
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      yMax=1,
      xi_start=1,
      k=1,
      yMin=0,
      Ti=60)
      annotation (Placement(transformation(extent={{-110,-76},{-94,-60}})));
    Modelica.Blocks.Math.Gain gain2(k=6) annotation (Placement(transformation(
          extent={{-7,-7},{7,7}},
          rotation=90,
          origin={195,-59})));
    RunningMeanTemperature6h runningMeanTemperature6h
      annotation (Placement(transformation(extent={{-268,-70},{-250,-50}})));
    Average6h average6h
      annotation (Placement(transformation(extent={{48,-118},{68,-98}})));
    IDEAS.Controls.SetPoints.Table tab(table=[-8.0 + 273,30 + 273; 15 + 273,22
           + 273])
      annotation (Placement(transformation(extent={{-210,-52},{-190,-32}})));
    IDEAS.Controls.SetPoints.Table tab1(table=[21 + 273,20 + 273; 22 + 273,27
           + 273])
      annotation (Placement(transformation(extent={{84,-100},{104,-80}})));
    Modelica.Blocks.Sources.Constant const1(k=1)
      annotation (Placement(transformation(extent={{154,-86},{170,-70}})));
    Modelica.Blocks.Math.Add sum2(k2=-1)
      annotation (Placement(transformation(extent={{186,-108},{206,-88}})));
    IDEAS.Fluid.FixedResistances.Junction jun(
      redeclare package Medium = Medium,
      dp_nominal={0,0,0},
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      m_flow_nominal={6,6,6},
      T_start=308.15)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=270,
          origin={76,-16})));
    IDEAS.Fluid.FixedResistances.Junction jun1(
      redeclare package Medium = Medium,
      m_flow_nominal={6,6,6},
      dp_nominal={0,0,0},
      energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-28,-28})));
    Modelica.Blocks.Logical.Switch switch2
      annotation (Placement(transformation(extent={{-154,-78},{-134,-58}})));
    Modelica.Blocks.Logical.Switch switch3
      annotation (Placement(transformation(extent={{118,-144},{138,-124}})));
    IDEAS.Controls.SetPoints.OccupancySchedule occSchWee1(period=7*24*3600,
        occupancy=3600*{7,19,31,43,55,67,79,91,103,115})        "Week schedule"
      annotation (Placement(transformation(extent={{-210,-80},{-190,-60}})));
    IDEAS.Controls.SetPoints.Table tab2(table=[-8 + 273,25 + 273; 15 + 273,17
           + 273])
      annotation (Placement(transformation(extent={{-210,-116},{-190,-96}})));
    IDEAS.Controls.SetPoints.Table tab3(table=[16 + 273,15 + 273; 17 + 273,22
           + 273])
      annotation (Placement(transformation(extent={{84,-174},{104,-154}})));
    IDEAS.Controls.SetPoints.OccupancySchedule occSchWee2(period=7*24*3600,
        occupancy=3600*{7,19,31,43,55,67,79,91,103,115})        "Week schedule"
      annotation (Placement(transformation(extent={{84,-138},{104,-118}})));
  equation
    connect(upscaleCase900_HVAC.heatPortEmb, embeddedPipe.heatPortEmb)
      annotation (Line(points={{-32,-96},{46,-96},{46,-80}}, color={191,0,0}));
    connect(fan2.port_b, vol.ports[2])
      annotation (Line(points={{14,52},{41.56,52},{41.56,56}},
                                                         color={0,127,255}));
    connect(fan2.port_a, heaPum.port_b1)
      annotation (Line(points={{-2,52},{-14,52}}, color={0,127,255}));
    connect(heaPum.port_a1, vol.ports[3])
      annotation (Line(points={{-14,32},{43,32},{43,56}}, color={0,127,255}));
    connect(bou.ports[1], heaPum.port_a2)
      annotation (Line(points={{-84,59},{-78,59},{-78,52},{-26,52}},
                                                   color={0,127,255}));
    connect(fan3.port_a, borFie.port_b) annotation (Line(points={{-78,-14},{-78,
            -28},{-76,-28}}, color={0,127,255}));
    connect(threeWayValveSwitch.port_b, fan3.port_b)
      annotation (Line(points={{-78,10},{-78,4}}, color={0,127,255}));
    connect(threeWayValveSwitch.port_a1, heaPum.port_a2) annotation (Line(
          points={{-78,26},{-78,52},{-26,52}}, color={0,127,255}));
    connect(threeWayValveSwitch.port_a2, hex.port_a1)
      annotation (Line(points={{-70,18},{2,18}}, color={0,127,255}));
    connect(temperatureSensor.port, vol.heatPort) annotation (Line(points={{34,87},
            {28,87},{28,64},{34,64}},     color={191,0,0}));
    connect(threeWayValveSwitch1.port_a2, hex.port_b2)
      annotation (Line(points={{68,18},{14,18}}, color={0,127,255}));
    connect(threeWayValveSwitch.switch, threeWayValveSwitch1.switch)
      annotation (Line(points={{-84.4,18},{-116,18},{-116,120},{88,120},{88,18},
            {82.4,18}}, color={255,0,255}));
    connect(realToBoolean.y, not1.u)
      annotation (Line(points={{-205.4,120},{-193.2,120}}, color={255,0,255}));
    connect(not1.y, threeWayValveSwitch1.switch) annotation (Line(points={{-179.4,
            120},{88,120},{88,18},{82.4,18}},        color={255,0,255}));
    connect(not2.u, hysteresis.y)
      annotation (Line(points={{166.6,65},{154.9,65}}, color={255,0,255}));
    connect(temperatureSensor.T, hysteresis.u) annotation (Line(points={{48,87},
            {90,87},{90,65},{134.2,65}}, color={0,0,127}));
    connect(not2.y, and1.u1) annotation (Line(points={{182.7,65},{190,65},{190,
            48},{102,48},{102,27},{142.6,27}}, color={255,0,255}));
    connect(and1.y, booleanToReal.u)
      annotation (Line(points={{158.7,27},{168.6,27}}, color={255,0,255}));
    connect(and1.u2, not3.y) annotation (Line(points={{142.6,21.4},{126.65,21.4},
            {126.65,7},{120.7,7}},   color={255,0,255}));
    connect(not3.u, threeWayValveSwitch1.switch) annotation (Line(points={{104.6,7},
            {96,7},{96,14},{88,14},{88,18},{82.4,18}},
                                                  color={255,0,255}));
    connect(realToInteger1.y, heaPum.stage) annotation (Line(points={{224.7,27},
            {228,27},{228,112},{-40,112},{-40,24},{-17,24},{-17,30}}, color={
            255,127,0}));
    connect(gain.y, fan2.m_flow_in)
      annotation (Line(points={{6,73.2},{6,61.6}}, color={0,0,127}));
    connect(gain.u, booleanToReal.y) annotation (Line(points={{6,91.6},{6,106},
            {198,106},{198,27},{184.7,27}}, color={0,0,127}));
    connect(realToInteger1.u, booleanToReal.y)
      annotation (Line(points={{208.6,27},{184.7,27}}, color={0,0,127}));
    connect(booleanToReal1.u, or1.y)
      annotation (Line(points={{-133.4,-5},{-137.3,-5}}, color={255,0,255}));
    connect(booleanToReal1.y, gain1.u)
      annotation (Line(points={{-117.3,-5},{-111.4,-5}}, color={0,0,127}));
    connect(gain1.y, fan3.m_flow_in)
      annotation (Line(points={{-95.3,-5},{-87.6,-5}}, color={0,0,127}));
    connect(or1.u1, booleanToReal.u) annotation (Line(points={{-153.4,-5},{-164,
            -5},{-164,108},{202,108},{202,14},{164,14},{164,27},{168.6,27}},
          color={255,0,255}));
    connect(heatingCoolingSet.SetpointHeating, realToBoolean.u) annotation (
        Line(points={{-226.9,119.88},{-217.45,119.88},{-217.45,120},{-219.2,120}},
          color={0,0,127}));
    connect(or1.u2, threeWayValveSwitch1.switch) annotation (Line(points={{-153.4,
            -10.6},{-170,-10.6},{-170,120},{88,120},{88,18},{82.4,18}},
          color={255,0,255}));
    connect(embeddedPipe.port_a, senTem.port_b)
      annotation (Line(points={{56,-70},{60,-70}}, color={0,127,255}));
    connect(fan1.port_b, senTem.port_a) annotation (Line(points={{76,-60},{76,
            -70},{74,-70}}, color={0,127,255}));
    connect(embeddedPipe.port_b, senTem1.port_a)
      annotation (Line(points={{36,-70},{34,-70}}, color={0,127,255}));
    connect(threeWayValveMotor.port_b, senTem1.port_b)
      annotation (Line(points={{20,-46},{20,-52},{16,-52},{16,-70},{20,-70}},
                                                   color={0,127,255}));
    connect(gain2.y, fan1.m_flow_in) annotation (Line(points={{195,-51.3},{88,
            -51.3},{88,-52},{85.6,-52}}, color={0,0,127}));
    connect(upscaleCase900_HVAC.TSensor[1], average6h.Input_average)
      annotation (Line(points={{-31.4,-108},{46,-108}},
          color={0,0,127}));
    connect(runningMeanTemperature6h.TRm, tab.u)
      annotation (Line(points={{-249.46,-60},{-236,-60},{-236,-42},{-212,-42}},
                                                        color={0,0,127}));
    connect(average6h.TRm, tab1.u)
      annotation (Line(points={{68.6,-108},{76,-108},{76,-90},{82,-90}},
                                                         color={0,0,127}));
    connect(vol.ports[4], threeWayValveSwitch1.port_a1) annotation (Line(points={{44.44,
            56},{76,56},{76,26}},        color={0,127,255}));
    connect(bou1.ports[1], threeWayValveSwitch1.port_a1)
      annotation (Line(points={{77,70},{76,70},{76,26}}, color={0,127,255}));
    connect(senTem.T, conPID1.u_m) annotation (Line(points={{67,-76.6},{66,
            -76.6},{66,-84},{-102,-84},{-102,-77.6}},  color={0,0,127}));
    connect(senTem1.T, conPID.u_m) annotation (Line(points={{27,-76.6},{27,-182},
            {164,-182},{164,-117.6}}, color={0,0,127}));
    connect(conPID1.y, threeWayValveMotor.ctrl) annotation (Line(points={{-93.2,
            -68},{11.36,-68},{11.36,-38}},                         color={0,0,
            127}));
    connect(threeWayValveMotor.port_a2, jun.port_3)
      annotation (Line(points={{28,-38},{58,-38},{58,-16},{66,-16}},
                                                   color={0,127,255}));
    connect(threeWayValveSwitch1.port_b, jun.port_1)
      annotation (Line(points={{76,10},{76,-6}}, color={0,127,255}));
    connect(fan1.port_a, jun.port_2)
      annotation (Line(points={{76,-44},{76,-26}}, color={0,127,255}));
    connect(heaPum.port_b2, jun1.port_3) annotation (Line(points={{-26,32},{-28,
            32},{-28,-18}}, color={0,127,255}));
    connect(borFie.port_a, jun1.port_2)
      annotation (Line(points={{-56,-28},{-38,-28}}, color={0,127,255}));
    connect(hex.port_b1, jun1.port_1) annotation (Line(points={{2,-2},{-8,-2},{
            -8,-28},{-18,-28}}, color={0,127,255}));
    connect(threeWayValveMotor.port_a1, hex.port_a2) annotation (Line(points={{
            20,-30},{18,-30},{18,-2},{14,-2}}, color={0,127,255}));
    connect(threeWayValveMotor.port_a1, vol.ports[5]) annotation (Line(points={
            {20,-30},{45.88,-30},{45.88,56}}, color={0,127,255}));
    connect(sum2.u1, const1.y) annotation (Line(points={{184,-92},{176,-92},{
            176,-78},{170.8,-78}}, color={0,0,127}));
    connect(tab.y, switch2.u1) annotation (Line(points={{-189,-42},{-178,-42},{
            -178,-60},{-156,-60}}, color={0,0,127}));
    connect(conPID1.u_s, switch2.y)
      annotation (Line(points={{-111.6,-68},{-133,-68}}, color={0,0,127}));
    connect(conPID.y, sum2.u2) annotation (Line(points={{172.8,-108},{178,-108},
            {178,-104},{184,-104}}, color={0,0,127}));
    connect(tab1.y, switch3.u1) annotation (Line(points={{105,-90},{110,-90},{
            110,-126},{116,-126}}, color={0,0,127}));
    connect(switch3.y, conPID.u_s) annotation (Line(points={{139,-134},{139,
            -118},{154.4,-118},{154.4,-108}}, color={0,0,127}));
    connect(tab2.y, switch2.u3) annotation (Line(points={{-189,-106},{-174,-106},
            {-174,-76},{-156,-76}}, color={0,0,127}));
    connect(tab2.u, runningMeanTemperature6h.TRm) annotation (Line(points={{
            -212,-106},{-236,-106},{-236,-60},{-249.46,-60}}, color={0,0,127}));
    connect(tab3.y, switch3.u3) annotation (Line(points={{105,-164},{110,-164},
            {110,-142},{116,-142}}, color={0,0,127}));
    connect(tab3.u, average6h.TRm) annotation (Line(points={{82,-164},{76,-164},
            {76,-108},{68.6,-108}}, color={0,0,127}));
    connect(sum2.y, gain2.u) annotation (Line(points={{207,-98},{222,-98},{222,
            -74},{195,-74},{195,-67.4}}, color={0,0,127}));
    connect(occSchWee2.occupied, switch3.u2)
      annotation (Line(points={{105,-134},{116,-134}}, color={255,0,255}));
    connect(occSchWee1.occupied, switch2.u2) annotation (Line(points={{-189,-76},
            {-184,-76},{-184,-74},{-178,-74},{-178,-68},{-156,-68}}, color={255,
            0,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-280,
              -220},{280,160}})),                                  Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-280,-220},{280,
              160}})),
      experiment(
        StopTime=31536000,
        __Dymola_NumberOfIntervals=15000,
        Tolerance=1e-06,
        __Dymola_fixedstepsize=20,
        __Dymola_Algorithm="Euler"),
      __Dymola_experimentSetupOutput,
      __Dymola_experimentFlags(
        Advanced(GenerateVariableDependencies=false, OutputModelicaCode=false),
        Evaluate=false,
        OutputCPUtime=false,
        OutputFlatModelica=false));
  end RBC_nightSetBack;
  annotation (uses(IDEAS(version="2.0.0"), Modelica(version="3.2.2")));
end Thesis_Bram_Stockman;
