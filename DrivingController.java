public class DrivingController {	
	public class DrivingCmd{
		public double steer;
		public double accel;
		public double brake;
		public int backward;
	};
	
	public double track_last_angle = 0.0;  // ���� Ʈ�� angle ����
	public double curr_whole_track_dist_straight = 0.0;  // ���� ���� Ʈ���� ��ü Ʈ�� ���� ����
	public double track_last_dist_straight = 0.0;  // ���� ���� Ʈ���� ���� ���� ����	
	public boolean isLogger = true;  // �׽�Ʈ �α׿� ���
	
	public DrivingCmd controlDriving(double[] driveArray, double[] aicarArray, double[] trackArray, double[] damageArray, int[] rankArray, int trackCurveType, double[] trackAngleArray, double[] trackDistArray, double trackCurrentAngle){
		DrivingCmd cmd = new DrivingCmd();
		
		////////////////////// input parameters
		double toMiddle     = driveArray[DrivingInterface.drvie_toMiddle    ];
		double angle        = driveArray[DrivingInterface.drvie_angle       ];
		double speed        = driveArray[DrivingInterface.drvie_speed       ];

		double toStart				 = trackArray[DrivingInterface.track_toStart		];
		double dist_track			 = trackArray[DrivingInterface.track_dist_track		];
		double track_width			 = trackArray[DrivingInterface.track_width			];
		double track_dist_straight	 = trackArray[DrivingInterface.track_dist_straight	];
		int track_curve_type		 = trackCurveType;

		double[] track_forward_angles	= trackAngleArray;
		double[] track_forward_dists	= trackDistArray;
		double track_current_angle		= trackCurrentAngle;
		
		double[] dist_cars = aicarArray;
		
		double damage		 = damageArray[DrivingInterface.damage];
		double damage_max	 = damageArray[DrivingInterface.damage_max];

		int total_car_num	 = rankArray[DrivingInterface.rank_total_car_num	];
		int my_rank			 = rankArray[DrivingInterface.rank_my_rank			];
		int opponent_rank	 = rankArray[DrivingInterface.rank_opponent_rank	];		
		////////////////////// END input parameters
		
		// To-Do : Make your driving algorithm
		System.out.println("====================== [start] =============================");
		
		double steer_angle = 0.0;
		double streer_coeff = 0.3;
		double corr_toMiddle = 0.0;
		double forward_ai_dist = 100.0;
		double emer_turn_yn = -1.0;
		double[] corr_route = new double[3]; 
		double corr_break = 0.0;
		double corr_accel = 0.25;
		double track_curve_level = 0.0;
		
		double[] user_change_dist = new double[2];
		
		// ���� Ʈ��Angle�� ���� Ʈ��Angle���� ���̷� Ŀ���� ���� �ľ��� ����.
		track_curve_level = Math.abs(track_current_angle - track_last_angle);
		if(track_last_dist_straight == 0.0 && track_dist_straight > 0.0) {
			curr_whole_track_dist_straight = track_dist_straight;
		} else {
			if(track_dist_straight == 0.0) {
				curr_whole_track_dist_straight = 0.0;
			}
		}
		
		// ������ angle ���
		double user_chagne_angle = angle;
		
		double[] forward_track_info = new double[2];
		forward_track_info = this.getForwardTrackInfo(speed, track_forward_angles, track_forward_dists, track_current_angle, toStart, angle, track_dist_straight, track_curve_type);		
		user_chagne_angle  = forward_track_info[0];
		
		if(isLogger) {			
			System.out.println("toStart : " + toStart);			
			System.out.println("track_width : " + track_width);
			System.out.println("track_last_angle : " + track_last_angle);
			System.out.println("track_curr_angle : " + track_current_angle);
			System.out.println("track_curve_level : " + track_curve_level);
			System.out.println("speed : " + speed);  // ��ġ�� �̻��ϰ� ����...
			System.out.println("toMiddle : " + toMiddle); // �����̸� �߾ӿ��� ������, ����̸� ����
			System.out.println("angle : " + angle);
			System.out.println("user_chagne_angle : " + user_chagne_angle);
			System.out.println("track_dist_straight : " + track_dist_straight); 
			System.out.println("track_last_dist_straight : " + track_last_dist_straight); 
			System.out.println("curr_whole_track_dist_straight : " + curr_whole_track_dist_straight);
			
			System.out.println("---------------------------------------------------");
		}
		
		angle = user_chagne_angle;
		
		// ������ �̵��� Ʈ���� �����ġ (+���� ������, -���� ����), ��ֹ��� ���� �̵��� �Ұ��� ��� -100 ����
		//corr_toMiddle = this.getCorrToMiddle(dist_cars, toMiddle, speed, angle, track_width, track_dist_straight, track_curve_type);
		corr_route[0] = -1.0;
		corr_route[1] = 0.0;
		corr_route[2] = 100.0;
		corr_route = this.getCorrToMiddle(dist_cars, toMiddle, speed, angle, track_width, track_dist_straight, track_curve_type);
		
		emer_turn_yn = corr_route[0];   // ��ֹ� ���ϱ� ���� ��� ������ ��� 0���� ū��
		corr_toMiddle = corr_route[1];  // �̵��� ��� (���� �߽����� ���� Ⱦ ����)
		forward_ai_dist = corr_route[2];  // �̵��� ��� (���� �߽����� ���� Ⱦ ����)
		
		if(emer_turn_yn > 1.0) {
			user_change_dist[0] = this.getBestDist(toMiddle, track_width, track_curve_type);
			user_change_dist = this.getChangeDist(dist_cars, toMiddle, user_change_dist[0], speed, track_width, track_dist_straight, track_curve_type);
		
			corr_toMiddle = user_change_dist[0];
			forward_ai_dist = user_change_dist[1];
		}
		
		double user_best_speed  = 100;
		
		/* ------------ �ӵ� ���� �Լ� �̰� -------------- */	
		// �ӵ� ���� �Լ�  <-- �̰��Լ�
		
		user_best_speed = this.getBestSpeed(angle, forward_ai_dist, speed, track_dist_straight, track_curve_type);
		
		// �극��ũ, ���� ���� <-- �̰��Լ�
		double[] user_speed_ctl = this.getSpeedCtl(speed, user_best_speed, track_dist_straight);
		
		double user_accelCtl = user_speed_ctl[0];  
		double user_breakCtl = user_speed_ctl[1];  
		/* ------------ �ӵ� ���� �Լ� �̰� -------------- */	
		
		
		// �극��ũ, ���� ���� <-- ��ü�Լ�(�ӽ÷� ���� Ŀ�� 10M ���� ������ �ӵ��� 110K �̻��� ��� �극��ŷ...Ŀ�꿡���� Ŀ���絵���� ������)
		double[] user_speed_ctl2 = this.getSpeedCtl2(speed, user_best_speed, track_dist_straight, track_curve_level);
		double user_accelCtl2 = user_speed_ctl2[0];  
		double user_breakCtl2 = user_speed_ctl2[1]; 
		/*-----------------------------------------*/
		
		// ���� ��ֹ��� �ִ� ��� �ӵ����� �Լ��� ���� ����
		if(emer_turn_yn > 1.0) {
			corr_accel = user_accelCtl; 
			corr_break = user_breakCtl;
		} else {
			corr_accel = user_accelCtl2; 
			corr_break = user_breakCtl2;
		}
		
		/* --- Ʈ�����ǿ� ���� angle��� �߰� �Լ� �ʿ� : �쿭å�� -- */
		// angle���� ���� ��� ���(�ӵ�, Ʈ���� ���ǿ� ���� ���)
//		streer_coeff = this.getSteerCoeff(speed, track_dist_straight);
		streer_coeff = this.getSteerCoeff2(track_current_angle, track_forward_angles, track_curve_type);
		
		// ���� ��ֹ��� �ִ� ��� 
		if(emer_turn_yn > 0.0) {
			streer_coeff = 1.0;
		}
		
		/* --- ����/���� �߰� �Լ� �ʿ� : ����å�� -- */		
		// ������ break ���� ó�� ��
		cmd.backward = DrivingInterface.gear_type_forward;
		
		if(toStart!=0 && speed<1){
			System.out.println("�������� �ƴϰ� ���ǵ尡 1�����ΰ��");
			
			if(Math.abs(toMiddle) > track_width/2){
				System.out.println("Ʈ���ۿ� �ִ� ���");
				if(toMiddle>0){
					System.out.println("�߾Ӽ� �����ʿ� �ִ� ���");
					corr_toMiddle = 0;
					streer_coeff -= 0.5;
					corr_accel += 0.5;
					
				}else{
					System.out.println("�߾Ӽ� ���ʿ� �ִ� ���");
					corr_toMiddle = 0;
					streer_coeff += 0.5;
					corr_accel += 0.5;
				}
				
			}else{
				System.out.println("Ʈ���ȿ� �ִ� ���");	
				if(toMiddle<0){
					System.out.println("�߾Ӽ� �����ʿ� �ִ� ���");
					if(dist_cars[0]<10){
						System.out.println("������ 10 ����");
						corr_toMiddle = corr_toMiddle + 4;
						user_best_speed = 100;
						speed += 20;
						streer_coeff -= 0.5;
						cmd.backward += 1;
						corr_accel += 1;
					}else{
						System.out.println("������ 10 �̻�");
						cmd.backward = 0;
					}
					
					
					
				}else{
					System.out.println("�߾Ӽ� ���ʿ� �ִ� ���");
					if(dist_cars[0]<10){
						System.out.println("������ 10 ����");
						corr_toMiddle = corr_toMiddle - 4;
						user_best_speed = 100;
						speed += 20;
						streer_coeff += 0.5;
						cmd.backward += 1;
						corr_accel += 1;
						
					}else{
						System.out.println("������ 10 �̻�");
						//cmd.backward = 0;
					}
					
					
				}
			}
			
		}
		
		if(isLogger) {
			System.out.println("emergency turn yn : " + emer_turn_yn);
		}
		// ������ ȸ���� angle�� ���
		steer_angle = this.getSteerAngle(angle, corr_toMiddle, track_width, streer_coeff);
		if(isLogger) {
			System.out.println("steer_angle : " + steer_angle);
			System.out.println("curr damage : " + damage + "(" + damage_max + ")");
		}
		this.track_last_angle = track_current_angle;
		this.track_last_dist_straight = track_dist_straight;
		/*-----------------------------------------*/
		
		////////////////////// output values		
		cmd.steer = steer_angle;
		cmd.accel = corr_accel; 
		cmd.brake = corr_break;
		
		////////////////////// END output values
		System.out.println("====================== [end] =============================");
		return cmd;
	}
	
	public static void main(String[] args) {
		DrivingInterface driving = new DrivingInterface();
		DrivingController controller = new DrivingController();
		
		double[] driveArray = new double[DrivingInterface.INPUT_DRIVE_SIZE];
		double[] aicarArray = new double[DrivingInterface.INPUT_AICAR_SIZE];
		double[] trackArray = new double[DrivingInterface.INPUT_TRACK_SIZE];
		double[] damageArray = new double[DrivingInterface.INPUT_DAMAGE_SIZE];
		int[] rankArray = new int[DrivingInterface.INPUT_RANK_SIZE];
		int[] trackCurveType = new int[1];
		double[] trackAngleArray = new double[DrivingInterface.INPUT_FORWARD_TRACK_SIZE];
		double[] trackDistArray = new double[DrivingInterface.INPUT_FORWARD_TRACK_SIZE];
		double[] trackCurrentAngle = new double[1];
				
		// To-Do : Initialize with your team name.
		int result = driving.OpenSharedMemory();
		
		if(result == 0){
			boolean doLoop = true;
			while(doLoop){
				result = driving.ReadSharedMemory(driveArray, aicarArray, trackArray, damageArray, rankArray, trackCurveType, trackAngleArray, trackDistArray, trackCurrentAngle);
				switch(result){
				case 0:
					DrivingCmd cmd = controller.controlDriving(driveArray, aicarArray, trackArray, damageArray, rankArray, trackCurveType[0], trackAngleArray, trackDistArray, trackCurrentAngle[0]);
					driving.WriteSharedMemory(cmd.steer, cmd.accel, cmd.brake, cmd.backward);
					break;
				case 1:
					break;
				case 2:
					// disconnected
				default:
					// error occurred
					doLoop = false;
					break;
				}
			}
		}
	}
	
	

	/**
	 * ���� Ʈ������ �̿� �Լ� 
	 * @param curr_track_forward_angles
	 * @param curr_track_forward_dists
	 * @param curr_track_current_angle
	 * @param curr_toStart
	 * @param curr_angle
	 * @param curr_track_dist_straight
	 * @param curr_track_curve_type
	 * @return
	 */
	private double[] getForwardTrackInfo(double curr_speed, double[] curr_track_forward_angles, double[] curr_track_forward_dists, double curr_track_current_angle, double curr_toStart, double curr_angle, double curr_track_dist_straight, double curr_track_curve_type){
		double[] forward_track_info = new double[2];
		forward_track_info[0] =  curr_angle; // ������ ���� angle ����
		forward_track_info[1] =  0; // �߰� ��� ����

		double user_forward_dists = 0;
		
		double track_chage_angle  = 0;
		double user_chagne_angle  = 0;
		long start_time = System.nanoTime();
		for(int i=0;i<20;i++){
			/* ���Ÿ� ��� */
			user_forward_dists = curr_track_forward_dists[i] - curr_toStart;
//			System.out.println("curr_speed["+i+"]   = " + curr_speed);
//			
//			System.out.println("track_forward_dists["+i+"]   = " + user_forward_dists);
//			System.out.println("track_current_angle     = " + track_current_angle*180/3.14);
//			System.out.println("track_forward_angles["+i+"] = " + track_forward_angles[i]);
//			System.out.println("track_forward_angles["+i+"]  = " + track_forward_angles[i]*180/3.14); 
			
			
			track_chage_angle = curr_track_current_angle - curr_track_forward_angles[i];			
				
//			System.out.println("track_chage_angle["+i+"]      = " + ( track_chage_angle*180/3.14 ));
//			System.out.println("track_chage_angle ����["+i+"]   = " + track_chage_angle);
//			System.out.println("track_current_angle ����["+i+"]  = " + ( curr_track_current_angle ));
//			System.out.println("track_forward_angles ����["+i+"] = " + curr_track_forward_angles[i]); 
			
			if(i == 0 && user_forward_dists < 3) // ���� 1m �̳��� ���(Ʈ���� �ټ� ����Ǵ� ��)
			{
				user_chagne_angle = curr_angle + track_chage_angle;
				forward_track_info[0] = user_chagne_angle;
//				System.out.println("user_chagne_angle ����["+i+"]    = " + user_chagne_angle);				
//				System.out.println();
				
			}else{
				forward_track_info[0] = curr_angle;
			}
			long end_time = System.nanoTime();
//			System.out.println("user_chagne_angle time " + (end_time-start_time));
			
			break; // ���� angle ������ ó���ϱ⶧���� �ѹ��� ����
		}
		
		
		return forward_track_info;		
	}
	
	/**
	 * �ڵ� Angle ����
	 * @param curr_angle
	 * @param curr_toMiddle
	 * @param curr_track_width
	 * @param streer_coeff
	 * @return
	 */
	private double getSteerAngle(double curr_angle, double curr_toMiddle, double curr_track_width, double streer_coeff) {
		double steer_angle = 0.0;
		
		steer_angle = streer_coeff * (curr_angle - curr_toMiddle/curr_track_width);
		
		return steer_angle;
	}
	
	/**
	 * �ڵ� Angle ����� ���
	 * @param curr_angle
	 * @param curr_toMiddle
	 * @param curr_track_width
	 * @param streer_coeff
	 * @return
	 */
	private double getSteerCoeff(double curr_speed, double curr_track_dist_straight) {
		double steer_coeff = 1.0;
		
		if(curr_track_dist_straight > 30) {
			if(curr_speed > 28.0) { 
				steer_coeff = 0.1;
			} else if( curr_speed > 25 && curr_speed <= 28.0) {
				steer_coeff = 0.3;
			} else if( curr_speed > 10 && curr_speed <= 25.0) {
				steer_coeff = 0.5;
			} else {
				steer_coeff = 1.0;
			}
		} else {
			steer_coeff = 1.0;
		}

		return steer_coeff;
	}
	
	private double getSteerCoeff2(double track_current_angle, double[] track_forward_angles, int track_curve_type){
		System.out.println("+++++++++++++++++++++++++++++++++++[steer coeff START]+++++++++++++++++++++++++++++++++++");
		double best_user_steer_coeff = 0.541052; // �ڵ���(Ʈ�������� ���� ����)
		
		double currRemain = 0.0;
		double currRemainAndForwardAngleSum = 0.0;
		double realRemain = 0.0;
		
		//��ȸ���� ���
		if(track_curve_type == 1){
			currRemain = ((double)3.14) - Math.abs(track_current_angle);
			currRemainAndForwardAngleSum = Math.abs(track_forward_angles[0]) + currRemain;
			realRemain = ((double)3.14) - currRemainAndForwardAngleSum;
			
//			if(isLogger){
//				System.out.println("���� Ʈ��angle�� �� ������                                      : " + currRemain);
//				System.out.println("���� ù��° angle                                               : " + Math.abs(track_forward_angles[0]));
//				System.out.println("���� Ʈ��angle�� �� �������� ���� ù��° angle�� ��             : " + currRemainAndForwardAngleSum);
//				System.out.println("���� Ʈ��angle�� �� �������� ���� ù��° angle�� ���� �� ������ : " + realRemain);
//			}
		}
		//��ȸ���� ���
		else if(track_curve_type == 2){
			currRemain = Math.abs(track_current_angle);
			currRemainAndForwardAngleSum = (((double)3.14) - Math.abs(track_forward_angles[0])) + currRemain;
			realRemain = ((double)3.14) - currRemainAndForwardAngleSum;
			
//			if(isLogger){
//				System.out.println("���� Ʈ��angle                                                  : " + currRemain);
//				System.out.println("���� ù��° angle                                               : " + Math.abs(track_forward_angles[0]));
//				System.out.println("���� Ʈ��angle�� ���� ù��° angle�� �� �������� ��             : " + currRemainAndForwardAngleSum);
//				System.out.println("���� Ʈ��angle�� ���� ù��° angle�� �� �������� ���� �� ������ : " + realRemain);
//			}
		}
		
		//��� ���� ��� : ���� Ʈ���� ���� ù��° Ʈ�� ���� ������ Ŭ���� ��Ŀ��� �Ǵ��ϰ� ����� �۰� ����
		//���� Ʈ���� ���� ù��° Ʈ���� ���� ������ ���.. ��� ������
		if(realRemain >= 0.0){
			if(realRemain < 0.1){
				best_user_steer_coeff = 1.0;
			}
			else if(realRemain < 0.2){
				best_user_steer_coeff = 0.9;
			}
			else if(realRemain < 0.3){
				best_user_steer_coeff = 0.8;
			}
			else if(realRemain < 0.4){
				best_user_steer_coeff = 0.7;
			}
			else if(realRemain < 0.5){
				best_user_steer_coeff = 0.6;
			}
			else if(realRemain < 0.6){
				best_user_steer_coeff = 0.5;
			}
			else if(realRemain < 0.7){
				best_user_steer_coeff = 0.4;
			}
			else if(realRemain < 0.8){
				best_user_steer_coeff = 0.3;
			}
			else if(realRemain < 0.9){
				best_user_steer_coeff = 0.2;
			}
			else if(realRemain < 1.0){
				best_user_steer_coeff = 0.1;
			}
			else{
				best_user_steer_coeff = 0.05;
			}
		}
		//���� Ʈ���� ���� ù��° Ʈ���� �ٸ� ������ ���.. ��� ������
		else{
			if(realRemain < -1.0){
				best_user_steer_coeff = 0.05;
			}
			else if(realRemain < -0.9){
				best_user_steer_coeff = 0.1;
			}
			else if(realRemain < -0.8){
				best_user_steer_coeff = 0.2;
			}
			else if(realRemain < -0.7){
				best_user_steer_coeff = 0.3;
			}
			else if(realRemain < -0.6){
				best_user_steer_coeff = 0.4;
			}
			else if(realRemain < -0.5){
				best_user_steer_coeff = 0.5;
			}
			else if(realRemain < -0.4){
				best_user_steer_coeff = 0.6;
			}
			else if(realRemain < -0.3){
				best_user_steer_coeff = 0.7;
			}
			else if(realRemain < -0.2){
				best_user_steer_coeff = 0.8;
			}
			else if(realRemain < -0.1){
				best_user_steer_coeff = 0.9;
			}
			else{
				best_user_steer_coeff = 1.0;
			}
		}
		
		System.out.println("best_user_steer_coeff : " + best_user_steer_coeff);
		System.out.println("+++++++++++++++++++++++++++++++++++[steer coeff end]+++++++++++++++++++++++++++++++++++");
		
		return best_user_steer_coeff;
	}
	
	/**
	 * ���� ��� �ĺ�
	 * @param curr_aicars
	 * @param curr_toMiddle
	 * @param curr_speed
	 * @param curr_angle
	 * @param curr_track_width
	 * @param curr_track_dist_straight
	 * @param curr_track_curve_type
	 * @return
	 */
	private double[] getCorrToMiddle(double[] curr_aicars, double curr_toMiddle, double curr_speed, double curr_angle, double curr_track_width, double curr_track_dist_straight, double curr_track_curve_type) {
		double corr_toMiddle = 0.0;
		double emer_turn_yn = 0.0;
		double tmp_ai_dist = 0.0;
		double fst_ai_dist = 0.0;
		double tmp_pre_ai_dist = 0.0;
		double tmp_ai_toMiddle = 0.0;
		double ai_car_width = 2.0;
		double ai_car_length = 4.0;
		double my_car_width = 2.0;
		double my_car_length = 4.5;
		double forward_dist_min = 2.0;
		double forward_dist_max = 80.0;
		double backward_dist_max = -10.0;
		double forward_ai_dist = 100.0;
		
		double[] ret_corr_route = new double[3];
		
		int[] tmp_r_ai = new int[10];
		int[] tmp_l_ai = new int[10];
		int[] tmp_c_ai = new int[10];
		int[] tmp_b_ai = new int[10];
		int tmp_r_ai_cnt = 0;
		int tmp_l_ai_cnt = 0;
		int tmp_c_ai_cnt = 0;
		int tmp_b_ai_cnt = 0;
		
		for(int i=0 ; i<10 ; i++){
			tmp_r_ai[i] = -1;
			tmp_l_ai[i] = -1;
			tmp_c_ai[i] = -1;
			tmp_b_ai[i] = -1;
		}
		
		ret_corr_route[0] = -1.0;
		ret_corr_route[1] = 0.0;
		ret_corr_route[2] = 100.0;
		
		// ��ֹ� ���� �迵�� ���� ����, ����, ���� �� ai ���� �迭 ����
		for(int i=1 ; i<curr_aicars.length ; i+=2) {
			//
			// ���� ���� ����������� ����
			tmp_ai_dist = this.getAiSideDist(curr_toMiddle, curr_aicars[i]);
						
			if(curr_aicars[i-1] > -100.0 && curr_aicars[i-1] < 100.0) {
				System.out.println("AI Car #" + (i+1)/2 + " : " + curr_aicars[i-1] + ", " + tmp_ai_dist);
			}
			
			
			// ó�� ��߽ô� ��� 0.0�̹Ƿ� ����
			if(curr_speed == 0.0){
				continue;
			}
			
			// �� ������ 10M �� 80M �տ� �ִ� ai������ �ϴ� ����
			if(curr_aicars[i-1] <= backward_dist_max || curr_aicars[i-1] >= forward_dist_max) {
				continue;
			}
			
			/*================ ������ ������ ���� ���� AI ���� �迭 ���� ====================*/
			// ������ ���� �浹 ��ġ ai ���� ��
			// ���� 4~50M, �������� 9M�� ���̿� �ִ� ������ ���� �������� ����
			if((curr_aicars[i-1] > forward_dist_min && curr_aicars[i-1] < forward_dist_max) 
					&& (tmp_ai_dist > -(my_car_width + 0.5) && tmp_ai_dist < (my_car_width + 0.5))) {
				tmp_c_ai[tmp_c_ai_cnt] = i;
				tmp_c_ai_cnt++;
//				System.out.println("   --> ���� ai ������ : " + tmp_c_ai_cnt);
			} else {
				
				// �� ���� ���ʿ� ��ġ�ϴ� ai������  �Ÿ������� array�� ����
				if(tmp_ai_dist < 0.0) {
					if(tmp_l_ai_cnt == 0) { // ù��° ���� ai������ �迭 �ε���
						tmp_l_ai[0] = i;
					} else {
						
						for(int j=0 ; j<10 ; j++) {
							if(tmp_l_ai[j] < 0) {
								tmp_l_ai[j] = i;
								break;
							} else {
								tmp_pre_ai_dist = this.getAiSideDist(curr_toMiddle, curr_aicars[tmp_l_ai[j]]);
								if(tmp_pre_ai_dist < tmp_ai_dist) {
									for(int k=j ; k < tmp_l_ai_cnt ; k++) {
										tmp_l_ai[k+1] = tmp_l_ai[k];
									}
									
									tmp_l_ai[j] = i;
								}
							}
							
							
						}
					}
					
					tmp_l_ai_cnt++;
					
//					System.out.println("   --> ���� ai ������ : " + tmp_l_ai_cnt);
				
				// �� ���� �����ʿ� ��ġ�ϴ� ai������ �Ÿ������� array�� ����	
				} else if (tmp_ai_dist >= 0.0) {
					if(tmp_r_ai_cnt == 0) { // ù��° ���� ai������ �迭 �ε���
						tmp_r_ai[0] = i;
					} else {
						
						for(int j=0 ; j<10 ; j++) {
							if(tmp_r_ai[j] < 0) {
								tmp_r_ai[j] = i;
								break;
							} else {
								
								tmp_pre_ai_dist = this.getAiSideDist(curr_toMiddle, curr_aicars[tmp_r_ai[j]]);
								
								if(tmp_pre_ai_dist < tmp_ai_dist) {
									for(int k=j ; k < tmp_r_ai_cnt ; k++) {
										tmp_r_ai[k+1] = tmp_r_ai[k];
									}
									
									tmp_r_ai[j] = i;
								}
							}
							
						}
					}
					
					tmp_r_ai_cnt++;
					
//					System.out.println("   --> ���� ai ������ : " + tmp_r_ai_cnt);
				}
			}
			
		} /* for�� �� */
		/*================ ������ ������ ���� �ٷ� ���� AI ���� �迭 ���� �� ====================*/
//		System.out.println("---------------------------------------------------");
		
		/*================ ��� ���� ====================*/
		// ai ������ ���ʿ� �ְ� �����ʿ� ���� ���
		if(tmp_r_ai_cnt == 0 && tmp_l_ai_cnt > 0) {
			if(isLogger) {
				System.out.println("Left ai car : " + tmp_l_ai_cnt + "," + tmp_c_ai_cnt);
			}
			
			// ���濡 ai ������ �ִ� ��� ������������
			//if(tmp_c_ai_cnt > 0  || curr_aicars[tmp_l_ai[0]] < 3.0) {
			if(tmp_c_ai_cnt > 0) {
				//corr_toMiddle = (curr_track_width/2 + curr_toMiddle)/2;
				corr_toMiddle = 3.0 + this.getAiSideDist(curr_toMiddle, curr_aicars[tmp_c_ai[0]]);  // ���� ��ֹ����� ������ 2.5M��� ����
				
				emer_turn_yn = 1.0;
				// Ʈ�� �ٱ����� ����� ��� Ʈ�������� ��� ����...���濡 �ִ� ��ֹ��� �ε��� ��쵵 �����ؾ���...
				if((curr_track_width/2 + curr_toMiddle) < corr_toMiddle ) {
					corr_toMiddle = curr_track_width/2  + curr_toMiddle;
					emer_turn_yn = 2.0;
				}
				
				
			} else {
				
				corr_toMiddle = this.getKeepTrackSideDist(curr_toMiddle, curr_track_width);
			}
		
	    // ai ������ �����ʿ� �ְ� ���ʿ� ���� ���
		} else if (tmp_r_ai_cnt > 0 && tmp_l_ai_cnt == 0) {
			if(isLogger) {
				System.out.println("Right ai car : " + tmp_r_ai_cnt + "," + tmp_c_ai_cnt);
			}
			
			// �ٷ� �տ� ai ������ �ִ� ��� ���ʹ�������
			//if(tmp_c_ai_cnt > 0 || curr_aicars[tmp_r_ai[0]] > -3.0) {
			if(tmp_c_ai_cnt > 0) {
				//corr_toMiddle = (-curr_track_width/2 + curr_toMiddle)/2;
				corr_toMiddle = -3.0 + this.getAiSideDist(curr_toMiddle, curr_aicars[tmp_c_ai[0]]);
				
				System.out.println("  �̵��� �Ÿ� : " + corr_toMiddle + ", Ʈ������ �Ÿ� : " + (-curr_track_width/2 + curr_toMiddle));
				emer_turn_yn = 1.0;
				// Ʈ�� �ٱ��� ���
				if((-curr_track_width/2 + curr_toMiddle) > corr_toMiddle ) {
					corr_toMiddle = -curr_track_width/2  + curr_toMiddle;
					
					emer_turn_yn = 2.0;
				}
				
				
				
			} else {
				
				corr_toMiddle = this.getKeepTrackSideDist(curr_toMiddle, curr_track_width);
			}
		
		// ���� ��� ������ �ִ� ���
		} else if (tmp_r_ai_cnt > 0 && tmp_l_ai_cnt > 0) {
			if(isLogger) {
				System.out.println("Left and Right ai car : " + tmp_l_ai_cnt + "," + tmp_r_ai_cnt + "," + tmp_c_ai_cnt);
			}
			
			double tmp_left_width = 0.0;
			double tmp_right_width = 0.0;
			// ���� ������ �ִ� ���
			if(tmp_c_ai_cnt > 0) {
				
				// �¿�, ���� ��� ������ �ִ� ��� ��/�� �� ������ ū ������ �߰����� ����
				tmp_left_width = this.getAiSideDist(curr_aicars[tmp_l_ai[0]],curr_aicars[tmp_c_ai[0]]);
				tmp_right_width = this.getAiSideDist(curr_aicars[tmp_c_ai[0]],curr_aicars[tmp_r_ai[0]]);
				
				emer_turn_yn = 1.0;
				
				if(tmp_left_width > tmp_right_width) {
					
					//������ �� ������ �������⿡ ����� ��츸 ����
					if(tmp_left_width > my_car_width) {
						corr_toMiddle = this.getAiSideDist(curr_toMiddle,curr_aicars[tmp_l_ai[0]]) + tmp_left_width/2;
						
					//������ �� ������ �������⿡ ������� ���� ���  ���� ������ �ٱ������� ����(������ ���� �� �켱 üũ)  
					} else {
						// ������ �ٷ� ���� �ִ��� üũ(�ٷ� ���� ����...�浹)
						if(curr_aicars[tmp_r_ai[0]-1] > my_car_length) {
							corr_toMiddle = this.getAiSideDist(curr_toMiddle,curr_aicars[tmp_r_ai[0]]) + my_car_width;
						} else if(curr_aicars[tmp_l_ai[0]-1] > my_car_length) {
							corr_toMiddle = this.getAiSideDist(curr_toMiddle,curr_aicars[tmp_l_ai[0]]) + my_car_width;
						} else {
							// ������ ��� �������� ���� �ִ� ��� ���� ������ 7M �տ� �ö������� ���� ������ ����
							if(curr_aicars[tmp_c_ai[0]-1] > 7.0) {
								corr_toMiddle = this.getKeepTrackSideDist(curr_toMiddle, curr_track_width);
							// ����, ������ ��� ������ �극��ŷ
							} else {
								
								corr_toMiddle = -100.0;
								emer_turn_yn = 2.0;
							}
						}
						
					}
				} else {
					//������ �� ������ �������⿡ ����� ��츸 ����
					if(tmp_right_width > my_car_width) {
						corr_toMiddle = this.getAiSideDist(curr_toMiddle,curr_aicars[tmp_r_ai[0]]) - tmp_right_width/2;
						
					//������ �� ������ �������⿡ ������� ���� ���  ���� ������ �ٱ������� ����(������ ���� �� �켱 üũ)  	
					} else {
						// ������ �ٷ� ���� �ִ��� üũ(�ٷ� ���� ����...�浹)
						if(curr_aicars[tmp_l_ai[0]-1] > my_car_length) {
							corr_toMiddle = this.getAiSideDist(curr_toMiddle,curr_aicars[tmp_l_ai[0]]) + my_car_width;
						} else if(curr_aicars[tmp_r_ai[0]-1] > my_car_length) {
							corr_toMiddle = this.getAiSideDist(curr_toMiddle,curr_aicars[tmp_r_ai[0]]) + my_car_width;
						} else {
							// ������ ��� �������� ���� �ִ� ��� ���� ������ 7M �տ� �ö������� ���� ������ ����
							if(curr_aicars[tmp_c_ai[0]-1] > 7.0) {
								corr_toMiddle = this.getKeepTrackSideDist(curr_toMiddle, curr_track_width);
							// ����, ������ ��� ������ �극��ŷ
							} else {
								corr_toMiddle = -100.0;
								emer_turn_yn = 2.0;
							}
						}
					}
				}
				

			// ���濡 ��ֹ������� ���� ��� ���� ��� ����
			} else {
				
				//corr_toMiddle = this.getAiSideDist(curr_toMiddle,curr_aicars[tmp_l_ai[0]]) + this.getAiSideDist(curr_aicars[tmp_l_ai[0],curr_aicars[tmp_r_ai[0]])/2;
				corr_toMiddle = this.getKeepTrackSideDist(curr_toMiddle, curr_track_width);
			}
			
		// ���ʿ� ai������ ���� ���	
		} else {
			
			double tmp_fst_forward_width = 0.0;
			
			// ���濡 ai������ �ִ� ���
			if(tmp_c_ai_cnt > 0) {
				if(isLogger) {
					System.out.println("Forward ai car : " + tmp_c_ai_cnt);
				}
				
				tmp_fst_forward_width = this.getAiSideDist(curr_toMiddle,curr_aicars[tmp_c_ai[0]]);
				
				
				if ( tmp_fst_forward_width < 0.0) { // ai ���� �߽��� ���� ������
				    //corr_toMiddle = (curr_track_width/2 + curr_toMiddle)/2;
					
					if(curr_angle < 0.0) {
						corr_toMiddle = tmp_fst_forward_width + my_car_width + 2.0;
					} else {
						corr_toMiddle = tmp_fst_forward_width + my_car_width + 1.0;
					}
					
					// Ʈ���� ����� ��� �ݴ��������
					if( ((curr_track_width-2)/2 + curr_toMiddle) < corr_toMiddle ) {
						
						// �����ַο��� 
						if (curr_track_dist_straight > 0.0 ) {
							corr_toMiddle = tmp_fst_forward_width - my_car_width - 2.0;
						} else {
							corr_toMiddle = (curr_track_width-2)/2 + curr_toMiddle;
						}
						//corr_toMiddle = curr_toMiddle;
					}
					
					
				} else {  // ai ���� �߽��� ������ ������

					
					if(curr_angle > 0.0) {
						corr_toMiddle = tmp_fst_forward_width - my_car_width - 2.0;
					} else {
						corr_toMiddle = tmp_fst_forward_width - my_car_width - 1.0;
					}

					// �����ַο��� Ʈ���� ����� ��� �ݴ�����
					if((-(curr_track_width-2)/2 + curr_toMiddle) > corr_toMiddle ) {
						
						// �����ַο��� 
						if (curr_track_dist_straight > 0.0 ) {
							corr_toMiddle = tmp_fst_forward_width + my_car_width + 2.0;
						} else {
							corr_toMiddle = -(curr_track_width-2)/2 + curr_toMiddle;
						}
	
						//corr_toMiddle = curr_toMiddle;
					}

				}
				
				emer_turn_yn = 1.0;

			} else {
				if(isLogger) {
					System.out.println("No ai car forward. Go Go!!!");
					System.out.println("track_curve_type : " + curr_track_curve_type);
					System.out.println("track_dist_straight : " + curr_track_dist_straight);
					System.out.println("track_whole_dist : " + curr_whole_track_dist_straight);
				}
				
				// ��ȸ�� �ڽ�
				if(curr_track_curve_type == 1.0) {
					if(isLogger) {
						System.out.println("Right Curve " + curr_track_dist_straight + " forward.");
					}
					
					//���� 10M �������� �������� ����
					if(curr_track_dist_straight > 15.0 && curr_whole_track_dist_straight > 50.0) {
						// ���� �� ���� ��ġ�� �߾Ӽ� ������ �ִ� ��츸 �������� ������ �ִ� ���� �߾Ӽ�����
						if(curr_toMiddle > 0) {
							corr_toMiddle = (-(curr_track_width-3)/2 + curr_toMiddle)/5;
							
							// Ʈ�� �ٱ��� ���
							if((-curr_track_width/2 + 1.5 + curr_toMiddle) > corr_toMiddle ) {
								corr_toMiddle = -curr_track_width/2  + curr_toMiddle + 2.0;
							}
							
						} else {
							corr_toMiddle = curr_toMiddle/5;
						}
					} else {
						if (curr_track_dist_straight > 2.0) {
							corr_toMiddle = ((curr_track_width - 2.0)/2 + curr_toMiddle)/curr_track_dist_straight;
						} else {
							corr_toMiddle = ((curr_track_width - 2.0)/2 + curr_toMiddle)/2;
						}
					}
					
				} else if(curr_track_curve_type == 2.0){
					if(isLogger) {
						System.out.println("Left Curve " + curr_track_dist_straight + " forward.");
					}
					
					//���� 10M �������� �������� ����
					if(curr_track_dist_straight > 15.0  && curr_whole_track_dist_straight > 50.0) {
						
						// ���� �� ���� ��ġ�� �߾Ӽ� ������ �ִ� ��츸 �������� ������ �ִ� ���� �߾Ӽ�����
						if(curr_toMiddle < 0) {
							corr_toMiddle = ((curr_track_width-3)/2 + curr_toMiddle)/5;
							
							// Ʈ�� �ٱ����� ����� ��� Ʈ�������� ��� ����...���濡 �ִ� ��ֹ��� �ε��� ��쵵 �����ؾ���...
							if((curr_track_width/2 - 1.5 + curr_toMiddle) < corr_toMiddle ) {
								corr_toMiddle = curr_track_width/2  + curr_toMiddle - 2.0;
							}
							
						} else {
							corr_toMiddle = curr_toMiddle/5;
						}
						
					} else {
						if (curr_track_dist_straight > 2.0) {
							corr_toMiddle = (-(curr_track_width - 2.0)/2 + curr_toMiddle)/curr_track_dist_straight ;
						} else {
							corr_toMiddle = (-(curr_track_width - 2.0)/2 + curr_toMiddle)/2;
						}
					}
				} else {
				
					corr_toMiddle = this.getKeepTrackSideDist(curr_toMiddle, curr_track_width);
				}
			}
		}

		
		ret_corr_route[0] = emer_turn_yn;
		ret_corr_route[1] = corr_toMiddle;
		if(tmp_c_ai_cnt > 0) {
			ret_corr_route[2] = curr_aicars[tmp_c_ai[0]-1]; // ���� ������ ���� ai ���� �Ÿ�
		}
		if(isLogger) {
			System.out.println("corr_toMiddle : " + corr_toMiddle);
		}

		return ret_corr_route;
	}
	
	/**
	 * ���� �� Mid������ ������ ���� ���ϱ� (���� ����)
	 * @param curr_toMiddle
	 * @param curr_aiMiddle
	 * @return
	 */
	private double getAiSideDist (double curr_toMiddle, double curr_aiMiddle) {
		double ret_dist = 0.0;
		
		ret_dist = curr_toMiddle - curr_aiMiddle ;
		
		return ret_dist;
	}
	
	/**
	 * Mid������ Ʈ�� ���̵���� ���� ���ϱ� (���� ����)
	 * @param curr_toMiddle
	 * @param curr_track_width
	 * @param side
	 * @return
	 */
	private double getTrackSideDist (double curr_toMiddle, double curr_track_width, int side) {
		double ret_dist = 0.0;
		
		if(side == 1) { // ������ ���̵���� �Ÿ�
			ret_dist = curr_track_width/2 + curr_toMiddle;
		} else if(side == 2) { // ���� ���̵���� �Ÿ�
			ret_dist = -(curr_track_width/2) + curr_toMiddle;
		}
		
		return ret_dist;
	}
	
	/**
	 * ���� ���� ��θ� �����ϱ� ���� �� (���� ����)
	 * 0���� �����ϸ� ���� ��ΰ� �����ǳ� ���� �̼�����
	 * @param curr_toMiddle
	 * @param curr_track_width
	 * @return
	 */
	private double getKeepTrackSideDist (double curr_toMiddle, double curr_track_width) {
		double ret_corr_toMiddle = 0.0;
		double tmp_r_track_side = this.getTrackSideDist(curr_toMiddle, curr_track_width, 1);
		double tmp_l_track_side = this.getTrackSideDist(curr_toMiddle, curr_track_width, 2);
		
		//���濡 ��ֹ� ������ ���� ���� �׳� ���� ���
		
		if(tmp_r_track_side < 0.0 || tmp_l_track_side > 0.0) {
			if(tmp_r_track_side < 0.0) {
				ret_corr_toMiddle = tmp_r_track_side - 1;
			}
			
			if(tmp_l_track_side > 0.0) {
				ret_corr_toMiddle = tmp_l_track_side + 1;
			}
			
		} 
		
		return ret_corr_toMiddle;
	}
	
	
	/**
	 *  ���� �ӵ� ���
	 * @param curr_speed
	 * @param curr_track_dist_straight
	 * @param curr_track_curve_type
	 * @return
	 */
	private double getBestSpeed(double curr_angle, double curr_dist_aicar, double curr_speed, double curr_track_dist_straight, double curr_track_curve_type){
		double user_best_speed = 100;
		
		float user_c_coeff      = (float)2.772;
		float user_d_coeff		= (float)-0.693;
		
		double curr_max_speed = 100;
		
		// 90 ~ 130 ���� �ϰ��  �ʴ� 40.2~58.1m �̵� : 
		// 70 ~  90  ���� �ϰ�� �ʴ� 31.2~40.2m �̵� : 
		//    ~  70  ���� �ϰ�� �ʴ� 0   ~31.2m �̵� : 
				
		if(curr_track_dist_straight > 100){
			curr_max_speed = 50;
			
			user_c_coeff = (float)2.772;
			user_d_coeff = (float)-0.693;
		}else if (curr_track_dist_straight > 80 && curr_track_dist_straight <= 100) {
			curr_max_speed = 40;
			
			user_c_coeff = (float)2.0;
			user_d_coeff = (float)-0.9;
		}else if (curr_track_dist_straight > 50 && curr_track_dist_straight <= 80) {
			curr_max_speed = 30;
			
			user_c_coeff = (float)1.5;
			user_d_coeff = (float)-1.2;
		}else{
			curr_max_speed = 25; // 90�� �̻��϶� ���� �ӵ�(88~90km/h)
			
			user_c_coeff = (float)1.0;
			user_d_coeff = (float)-1.5;
		}
		
		
		
		double curr_angle_abs = Math.abs(curr_angle*180/3.14);
		
		if(curr_angle_abs <= 10){
			curr_max_speed = curr_max_speed*1.3;
		}else if(curr_angle_abs > 10 && curr_angle_abs <= 30 ){
			curr_max_speed = curr_max_speed*1;
		}else if(curr_angle_abs > 30 && curr_angle_abs <= 45 ){
			curr_max_speed = curr_max_speed*0.7;
		}else if(curr_angle_abs > 45 && curr_angle_abs <= 90 ){		
			curr_max_speed = curr_max_speed*0.6;
		}else if(curr_angle_abs > 90 && curr_angle_abs <= 135 ){
			curr_max_speed = curr_max_speed*0.5;
		}else if(curr_angle_abs > 135 && curr_angle_abs <= 180 ){
			curr_max_speed = curr_max_speed*0.2;
		}else{
			curr_max_speed = curr_max_speed*0.1;
		}
		 
//		/*�׽�Ʈ ���� */
		//curr_max_speed = 25;
//		/*�׽�Ʈ ���� */
		
		user_best_speed = curr_max_speed * (1 - Math.exp(-user_c_coeff/curr_max_speed * curr_dist_aicar - user_d_coeff));
		if(isLogger) {
			System.out.println("+++++++++++++++++ ���� �ӵ� ���[start] ++++++++++++++++++++++");
			System.out.println("curr_max_speed          ="+curr_max_speed);
			System.out.println("user_best_speed         ="+user_best_speed);
			System.out.println("curr_speed              ="+curr_speed + " m/s");
			System.out.println("curr_speed              ="+curr_speed*3.6 + " km/h");
			System.out.println("curr_angle              ="+curr_angle);
			System.out.println("curr_angle_abs          ="+curr_angle_abs + " ��");
			System.out.println("curr_track_dist_straight="+curr_track_dist_straight);
			System.out.println("+++++++++++++++++ ���� �ӵ� ���[end] ++++++++++++++++++++++");
		}
		return user_best_speed;
	}
	
	/**
	 * �극��ũ, ���� ���� �Լ�
	 * @param curr_speed
	 * @param curr_best_speed
	 * @param curr_track_dist_straight
	 * @return
	 */
	private double[] getSpeedCtl(double curr_speed, double curr_best_speed, double curr_track_dist_straight){
		double[] user_speed_ctl = new double[2];
		user_speed_ctl[0] = 0.2; // accel
		user_speed_ctl[1] = 0.0; // brake

		if(curr_speed > curr_best_speed) {
			if(isLogger) {
				System.out.println("+++++++++++++++++ �극��ũ, ���� ���� �Լ�[start] ++++++++++++++++++++++");
				System.out.println("curr_speed               = "+curr_speed);
				System.out.println("curr_best_speed          = "+curr_best_speed);
				System.out.println("curr_track_dist_straight = "+curr_track_dist_straight);
			}
			user_speed_ctl[0] = 0.1;
			
			if(curr_track_dist_straight < 20){
				user_speed_ctl[1] = 0.2;
			}else{
				user_speed_ctl[1] = 0.2;
			}
			if(isLogger) {
				System.out.println("user_brakeCtl="+user_speed_ctl[1]);
				System.out.println("+++++++++++++++++ �극��ũ, ���� ���� �Լ�[end] ++++++++++++++++++++++");
			}
			
		}else{
			user_speed_ctl[0] = 0.4;
		}		

		
		return user_speed_ctl;		
	}
	
	/**
	 * �극��ũ, ���� ���� �Լ�(��ü)
	 * @param curr_speed
	 * @param curr_best_speed
	 * @param curr_track_dist_straight
	 * @return
	 */
	private double[] getSpeedCtl2(double curr_speed, double curr_best_speed, double curr_track_dist_straight, double track_curve_level){
		double corr_break = 0.0;
		double corr_accel = 0.2;
		double[] user_speed_ctl = new double[2];
		user_speed_ctl[0] = 0.2; // accel
		user_speed_ctl[1] = 0.0; // brake

		//if(curr_speed > curr_best_speed) {
			if(isLogger) {
				System.out.println("+++++++++++++++++ �극��ũ, ���� ���� �Լ�2[start] ++++++++++++++++++++++");
				System.out.println("curr_speed               = "+curr_speed);
				System.out.println("curr_best_speed          = "+curr_best_speed);
				System.out.println("curr_track_dist_straight = "+curr_track_dist_straight);
			}
			
			if(curr_track_dist_straight > 0.0 && curr_track_dist_straight < 15.0){
				if( curr_speed > 35.0) {
					corr_break = 0.4;
					corr_accel = 0.1;
				} else if( curr_speed > 30.0  && curr_speed <= 35.0) {
					corr_break = 0.3;
					corr_accel = 0.1;
				} else if ( curr_speed > 23.0 && curr_speed <= 30.0){
					corr_break = 0.2;
					corr_accel = 0.1;
				} else if ( curr_speed > 14.0 && curr_speed <= 23.0){
					corr_break = 0.1;
					corr_accel = 0.1;
				} else {
					corr_break = 0.1;
					corr_accel = 0.2;				
				}
			} else if(curr_track_dist_straight == 0.0) {
				if( curr_speed > 35.0) {
					if(track_curve_level > 0.055) {
						corr_break = 0.5;
						corr_accel = 0.1;
					} else if(track_curve_level > 0.045 && track_curve_level <= 0.055) {
						corr_break = 0.4;
						corr_accel = 0.1;
					} else if (track_curve_level > 0.035 && track_curve_level <= 0.045) {
						corr_break = 0.3;
						corr_accel = 0.1;
					} else if (track_curve_level > 0.030 && track_curve_level <= 0.035) {
						corr_break = 0.2;
						corr_accel = 0.1;
					} else {
						corr_break = 0.1;
						corr_accel = 0.1;
					}
					
				} else if( curr_speed > 30.0  && curr_speed <= 35.0) {
					if(track_curve_level > 0.055) {
						corr_break = 0.4;
						corr_accel = 0.1;
					} else if(track_curve_level > 0.045 && track_curve_level <= 0.055) {
						corr_break = 0.3;
						corr_accel = 0.1;
					} else if (track_curve_level > 0.035 && track_curve_level <= 0.045) {
						corr_break = 0.2;
						corr_accel = 0.1;
					} else if (track_curve_level > 0.030 && track_curve_level <= 0.035) {
						corr_break = 0.1;
						corr_accel = 0.1;
					} else {
						corr_break = 0.1;
						corr_accel = 0.2;
					}
				} else if ( curr_speed > 23.0 && curr_speed <= 30.0){
					
					if(track_curve_level > 0.055) {
						corr_break = 0.3;
						corr_accel = 0.1;
					} else if(track_curve_level > 0.045 && track_curve_level <= 0.055) {
						corr_break = 0.2;
						corr_accel = 0.1;
					} else if (track_curve_level > 0.035 && track_curve_level <= 0.045) {
						corr_break = 0.1;
						corr_accel = 0.1;
					} else if (track_curve_level > 0.030 && track_curve_level <= 0.035) {
						corr_break = 0.0;
						corr_accel = 0.1;
					} else {
						corr_break = 0.0;
						corr_accel = 0.2;
					}
				} else if ( curr_speed > 14 && curr_speed <= 23.0){
					if(track_curve_level > 0.055) {
						corr_break = 0.2;
						corr_accel = 0.1;
					} else if(track_curve_level > 0.045 && track_curve_level <= 0.055) {
						corr_break = 0.1;
						corr_accel = 0.1;
					} else if (track_curve_level > 0.035 && track_curve_level <= 0.045) {
						corr_break = 0.0;
						corr_accel = 0.1;
					} else if (track_curve_level > 0.030 && track_curve_level <= 0.035) {
						corr_break = 0.0;
						corr_accel = 0.2;
					} else {
						corr_break = 0.0;
						corr_accel = 0.3;
					}
					
				} else {
					if(track_curve_level > 0.055) {
						corr_break = 0.1;
						corr_accel = 0.1;
					} else if(track_curve_level > 0.045 && track_curve_level <= 0.055) {
						corr_break = 0.1;
						corr_accel = 0.2;
					} else if (track_curve_level > 0.035 && track_curve_level <= 0.045) {
						corr_break = 0.0;
						corr_accel = 0.2;
					} else if (track_curve_level > 0.030 && track_curve_level <= 0.035) {
						corr_break = 0.0;
						corr_accel = 0.3;
					} else {
						corr_break = 0.0;
						corr_accel = 0.4;
					}
									
				}
				
			} else {
				if( curr_speed > 35.0) {
					if(curr_track_dist_straight > 100) {
						corr_break = 0.0;
						corr_accel = 0.4;
					} else {
						corr_break = 0.0;
						corr_accel = 0.3;
					}
				} else if ( curr_speed > 23 && curr_speed <= 35.0){
					if(curr_track_dist_straight > 100) {
						corr_break = 0.0;
						corr_accel = 0.5;
					} else {
						corr_break = 0.0;
						corr_accel = 0.4;
					}
				} else {
					if(curr_track_dist_straight > 100) {
						corr_break = 0.0;
						corr_accel = 0.6;
					} else {
						corr_break = 0.0;
						corr_accel = 0.5;
					}
				}
			}
			
			user_speed_ctl[0] = corr_accel;
			user_speed_ctl[1] = corr_break;
			if(isLogger) {
				System.out.println("user_accelCtl="+user_speed_ctl[0]);
				System.out.println("user_brakeCtl="+user_speed_ctl[1]);
				System.out.println("+++++++++++++++++ �극��ũ, ���� ���� �Լ�2[end] ++++++++++++++++++++++");
			}
			
		//}else{
		//	user_speed_ctl[0] = 0.4;
		//}		

		
		return user_speed_ctl;		
	}
	
	/**
	 * ���� �̵� �Լ�(�⺻)
	 * @param curr_toMiddle
	 * @param curr_track_width
	 * @param curr_track_curve_type
	 * @return
	 */
	private double getBestDist(double curr_toMiddle, double curr_track_width, double curr_track_curve_type){
		double user_change_dist = 0.0;		
		
		if(curr_track_curve_type == 1){ // ��ȸ�� 
			user_change_dist = curr_toMiddle + ( curr_track_width/3 );  
		}else if(curr_track_curve_type == 2){ // ��ȸ��
			user_change_dist = curr_toMiddle - ( curr_track_width/3 );
		}else{
			user_change_dist = curr_toMiddle;
		}		
		
		return user_change_dist;		
	}	
	
	/**
	 * ��ֹ� ó�� �Լ�
	 * @param curr_aicars
	 * @param change_toMiddle
	 * @param curr_speed
	 * @param curr_track_width
	 * @param curr_track_dist_straight
	 * @param curr_track_curve_type
	 * @return
	 */
	private double[] getChangeDist(double[] curr_aicars, double curr_toMiddle, double change_toMiddle, double curr_speed, double curr_track_width, double curr_track_dist_straight, double curr_track_curve_type) {
		

		
		double user_check_forward = 100;
		double user_car_width = 2; // ���� 2m
		double user_car_width_half = user_car_width/2; // ���� 2m
		double ai_car_length      = 4.8; // ���� 4.8
		double ai_car_length_half = ai_car_length/2; // ���� 4.8
		
		if(curr_speed >= 30){
			user_check_forward = 100;
		}else if(curr_speed < 30 && curr_speed >= 20){
			user_check_forward = 50;
		}else{
			user_check_forward = 10;
		}		
		
		System.out.println("user_check_forward         = "+user_check_forward);
		
//		/*�׽�Ʈ ���� */
//		user_check_forward = 20;
//		/*�׽�Ʈ ���� */

		
		double[] user_change_dist = new double[2]; // change_toMiddle;
		user_change_dist[0] = change_toMiddle; // ������ �Ÿ� (�ʱ�� ����ȭ�� �ڽ��� ����)
		user_change_dist[1] = 100; // ���� ��ֹ� �Ÿ�
		
		// ���� ���� ��� ����
		double[] cars_dist = new double[2];
		cars_dist[0] = 2; // ���� �ǰŸ�
		cars_dist[1] = 2; // ���� �߽ɰŸ�(�¿� �Ǵ� ����)
		
		for(int i=0;i<5;i++){
			System.out.println("dist_cars_dist[" + i + "]    = " + curr_aicars[i]);
			System.out.println("dist_cars_middle[" + i + "]  = " + curr_aicars[i + 1]);
			
			if(i == 0 && curr_aicars[i] == 100){
				System.out.println("���� ��ֹ����� ����");
				break;
			}else{
				if(curr_aicars[i] == 100 || curr_aicars[i] > user_check_forward) {
					System.out.println("���� ����[" + i + "] �� ����");
					continue;
				}
			}
			
			
			
			cars_dist = this.getCarsDist(curr_toMiddle, change_toMiddle, curr_aicars[2*i + 1], curr_track_curve_type, user_car_width, ai_car_length);
			
			
			// ���� �߽ɰŸ��� ���� �̸� ������ �浹�ϴ� ������ �Ǵ��Ͽ� ��ȸ
			// TODO : ���� ���� �ʿ� ���� �ǰŸ�
			if(cars_dist[0] <=  1.2 ){
				System.out.println("+++++++++++++++++ ���� ��ֹ� �߰� ��ȸ �ϼ��� ++++++++++++++++++++++");
				System.out.println("���� ����[" + i + "] �� ȸ��");
				if(cars_dist[1] > 0){ // ��ֹ� ���� ������ ����(�������� �⺻ �̵�)
					change_toMiddle = change_toMiddle + ( Math.abs(cars_dist[0]) + user_car_width );
				}else if(cars_dist[1] < 0){ // ��ֹ� ���� ������ ����(���������� �⺻ �̵�)
					change_toMiddle = change_toMiddle - ( Math.abs(cars_dist[0]) + user_car_width );
				}else{
					if(curr_track_curve_type == 1 ){ // ��ȸ�� �ڽ�(���������� �⺻ �̵�)
						change_toMiddle = change_toMiddle - ( Math.abs(cars_dist[0]) + user_car_width );
					}else{ // ��ȸ�� �ڽ�(�������� �⺻ �̵�)
						change_toMiddle = change_toMiddle + ( Math.abs(cars_dist[0]) + user_car_width );
					}
				}
				user_change_dist[1] = curr_aicars[i];
				break;
			}else if(cars_dist[0] > 1.2 && cars_dist[0] <= 1.5) {
				System.out.println("+++++++++++++++++ ���� ��ֹ� �߰� ���� �ϼ��� ++++++++++++++++++++++");
				System.out.println("�¿� ����[" + i + "] Ȯ�� �ʿ�");
				if(cars_dist[1] > 0){ // ������ ����
					change_toMiddle = change_toMiddle + Math.abs(cars_dist[0]);
				}else if(cars_dist[1] < 0){ // ������ ����
					change_toMiddle = change_toMiddle - Math.abs(cars_dist[0]);
				}else{
					if(curr_track_curve_type == 1 ){ // ��ȸ�� �ڽ�
						change_toMiddle = change_toMiddle + Math.abs(cars_dist[0]);
					}else{ // ��ȸ�� �ڽ�
						change_toMiddle = change_toMiddle - Math.abs(cars_dist[0]);
					}
				}
			}			
		}
		
		user_change_dist[0] = change_toMiddle;
		
		System.out.println("change_toMiddle         = "+change_toMiddle);
		System.out.println("user_change_middle      = "+user_change_dist[0]);
		System.out.println("user_ai_car_dist        = "+user_change_dist[1]);
		
				
		return user_change_dist;
	}
	
	/**
	 * ������ �ǰŸ� ���
	 * @param user_car_toMiddle
	 * @param ai_car_toMiddle
	 * @param curr_track_curve_type
	 * @param user_car_width_half
	 * @param ai_car_length_half
	 * @return
	 */
	private double[] getCarsDist(double curr_toMiddle, double user_change_toMiddle, double ai_car_toMiddle, double curr_track_curve_type, double user_car_width, double ai_car_length){
		double[] cars_dist = new double[2];
		cars_dist[0] =  0; // ������ �ǰŸ�
		cars_dist[1] =  0; // ���� �߽ɰŸ�(�¿� �Ǵ� ����)
		
		double cars_middle_dist = curr_toMiddle - ai_car_toMiddle; // �������� �߽� �Ÿ�
		cars_dist[1] = cars_middle_dist;
		System.out.println("cars_middle_dist(�߽ɰŸ�) = "+cars_middle_dist); 
		
		// ������ �ǰŸ� (������ ����Ͽ� ���)
		cars_dist[0] = Math.abs(cars_middle_dist) - user_car_width/2 - ai_car_length/2;		
		System.out.println("cars_dist(�ǰŸ�)         = "+cars_dist[0]);
		
		return cars_dist;		
	}
	
			
}