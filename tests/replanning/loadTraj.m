
    
    [time,com_x] =  textread(strcat('./replan_data/com_x',num2str(plan)), format_input);
    [time,com_y] =  textread(strcat('./replan_data/com_y',num2str(plan)), format_input);
        
    [time,com_xd] =  textread(strcat('./replan_data/com_xd',num2str(plan)), format_input);
    [time,com_yd] =  textread(strcat('./replan_data/com_yd',num2str(plan)), format_input);
        
    [time,zmp_x] =  textread(strcat('./replan_data/zmp_x',num2str(plan)), format_input);
    [time,zmp_y] =  textread(strcat('./replan_data/zmp_y',num2str(plan)), format_input);

    

    [step, footHoldsLF(:,1),footHoldsLF(:,2)] = textread(strcat('./replan_data/footHoldsLF',num2str(plan)), format_input2);
    [step,footHoldsRF(:,1), footHoldsRF(:,2)] = textread(strcat('./replan_data/footHoldsRF',num2str(plan)), format_input2);
    [step,footHoldsLH(:,1), footHoldsLH(:,2)] = textread(strcat('./replan_data/footHoldsLH',num2str(plan)), format_input2);
    [step,footHoldsRH(:,1), footHoldsRH(:,2)] = textread(strcat('./replan_data/footHoldsRH',num2str(plan)), format_input2);

    swing=[];
    [time,swing(1,:)] = textread(strcat('./replan_data/swingLF',num2str(plan)), format_input);
    [time,swing(2,:)] = textread(strcat('./replan_data/swingRF',num2str(plan)), format_input);
    [time,swing(3,:)] = textread(strcat('./replan_data/swingLH',num2str(plan)), format_input);
    [time,swing(4,:)] = textread(strcat('./replan_data/swingRH',num2str(plan)), format_input);
    
    [time,   footPos_x(1,:),footPos_y(1,:)] = textread(strcat('./replan_data/footPosLF',num2str(plan)), format_input2);
    [time,   footPos_x(2,:),footPos_y(2,:)] = textread(strcat('./replan_data/footPosRF',num2str(plan)), format_input2);
    [time,   footPos_x(3,:),footPos_y(3,:)] = textread(strcat('./replan_data/footPosLH',num2str(plan)), format_input2);
    [time,   footPos_x(4,:),footPos_y(4,:)] = textread(strcat('./replan_data/footPosRH',num2str(plan)), format_input2);
  

	[time_exp, jerk_disturbance] = textread('./replan_data/jerk_disturbance', format_input);

