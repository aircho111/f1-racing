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
		
		double steer_angle = 0.0;
		double streer_coeff = 0.3;
		double corr_toMiddle = 0.0;
		double forward_ai_dist = 100.0;
		double emer_turn_yn = -1.0;
		double[] corr_route = new double[3]; 
		double corr_break = 0.0;
		double corr_accel = 0.25;
		double track_curve_level = 0.0;
		
		// ���� Ʈ��Angle�� ���� Ʈ��Angle���� ���̷� Ŀ���� ���� �ľ��� ����.
		track_curve_level = Math.abs(track_current_angle - track_last_angle);
		if(track_last_dist_straight == 0.0 && track_dist_straight > 0.0) {
			curr_whole_track_dist_straight = track_dist_straight;
		} else {
			if(track_dist_straight == 0.0) {
				curr_whole_track_dist_straight = 0.0;
			}
		}
		
		System.out.println("===================================================");
		System.out.println("toStart : " + toStart);
		
		System.out.println("track_width : " + track_width);
		System.out.println("track_last_angle : " + track_last_angle);
		System.out.println("track_curr_angle : " + track_current_angle);
		System.out.println("track_curve_level : " + track_curve_level);
		System.out.println("speed : " + speed);  // ��ġ�� �̻��ϰ� ����...
		System.out.println("toMiddle : " + toMiddle); // �����̸� �߾ӿ��� ������, ����̸� ����
		System.out.println("angle : " + angle);
		System.out.println("track_dist_straight : " + track_dist_straight); 
		System.out.println("track_last_dist_straight : " + track_last_dist_straight); 
		System.out.println("curr_whole_track_dist_straight : " + curr_whole_track_dist_straight);
		
		System.out.println("---------------------------------------------------");
		
		// ������ �̵��� Ʈ���� �����ġ (+���� ������, -���� ����), ��ֹ��� ���� �̵��� �Ұ��� ��� -100 ����
		//corr_toMiddle = this.getCorrToMiddle(dist_cars, toMiddle, speed, angle, track_width, track_dist_straight, track_curve_type);
		corr_route[0] = -1.0;
		corr_route[1] = 0.0;
		corr_route[2] = 100.0;
		corr_route = this.getCorrToMiddle(dist_cars, toMiddle, speed, angle, track_width, track_dist_straight, track_curve_type);
		
		emer_turn_yn = corr_route[0];   // ��ֹ� ���ϱ� ���� ��� ������ ��� 0���� ū��
		corr_toMiddle = corr_route[1];  // �̵��� ��� (���� �߽����� ���� Ⱦ ����)
		forward_ai_dist = corr_route[2];  // �̵��� ��� (���� �߽����� ���� Ⱦ ����)
		
		/* --- ����/���� �߰� �Լ� �ʿ� : ����å�� -- */		
		// ������ break ���� ó�� ��
		
		/* ------------ �ӵ� ���� �Լ� �̰� -------------- */	
		// �ӵ� ���� �Լ�  <-- �̰��Լ�
		double user_best_speed  = 100;
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
		
		
		/* --- Ʈ�����ǿ� ���� angle��� �߰� �Լ� �ʿ� : �쿭å�� -- */
		// angle���� ���� ��� ���(�ӵ�, Ʈ���� ���ǿ� ���� ���)
		streer_coeff = this.getSteerCoeff(speed, track_dist_straight);
		
		if(emer_turn_yn > 0.0) {
			streer_coeff = 1.0;
		}
		
		System.out.println("emergency turn yn : " + emer_turn_yn);
		
		// ������ ȸ���� angle�� ���
		steer_angle = this.getSteerAngle(angle, corr_toMiddle, track_width, streer_coeff);
		
		System.out.println("steer_angle : " + steer_angle);
		System.out.println("curr damage : " + damage + "(" + damage_max + ")");
		
		this.track_last_angle = track_current_angle;
		this.track_last_dist_straight = track_dist_straight;
		/*-----------------------------------------*/
		
		////////////////////// output values		
		cmd.steer = steer_angle;
		
		// ���� ��ֹ��� �ִ� ��� �ӵ����� �Լ��� ���� ����
		if(emer_turn_yn > 0) {
			cmd.accel = user_accelCtl; 
			cmd.brake = user_breakCtl;
		} else {
			cmd.accel = user_accelCtl2; 
			cmd.brake = user_breakCtl2;
		}
		cmd.backward = DrivingInterface.gear_type_forward;
		////////////////////// END output values
		
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
						
//			if(curr_aicars[i-1] > -100.0 && curr_aicars[i-1] < 100.0) {
//				System.out.println("AI Car #" + (i+1)/2 + " : " + curr_aicars[i-1] + ", " + tmp_ai_dist);
//			}
			
			
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
			System.out.println("Left ai car : " + tmp_l_ai_cnt + "," + tmp_c_ai_cnt);
			
			// ���濡 ai ������ �ִ� ��� ������������
			//if(tmp_c_ai_cnt > 0  || curr_aicars[tmp_l_ai[0]] < 3.0) {
			if(tmp_c_ai_cnt > 0) {
				//corr_toMiddle = (curr_track_width/2 + curr_toMiddle)/2;
				corr_toMiddle = 2.5 + this.getAiSideDist(curr_toMiddle, curr_aicars[tmp_c_ai[0]]);  // ���� ��ֹ����� ������ 2.5M��� ����
				
				// Ʈ�� �ٱ����� ����� ��� Ʈ�������� ��� ����...���濡 �ִ� ��ֹ��� �ε��� ��쵵 �����ؾ���...
				if((curr_track_width/2 + curr_toMiddle) < corr_toMiddle ) {
					corr_toMiddle = curr_track_width/2  + curr_toMiddle + 1.0;
				}
				
				emer_turn_yn = 1.0;
			} else {
				
				corr_toMiddle = this.getKeepTrackSideDist(curr_toMiddle, curr_track_width);
			}
		
	    // ai ������ �����ʿ� �ְ� ���ʿ� ���� ���
		} else if (tmp_r_ai_cnt > 0 && tmp_l_ai_cnt == 0) {
			System.out.println("Right ai car : " + tmp_r_ai_cnt + "," + tmp_c_ai_cnt);
			
			// �ٷ� �տ� ai ������ �ִ� ��� ���ʹ�������
			//if(tmp_c_ai_cnt > 0 || curr_aicars[tmp_r_ai[0]] > -3.0) {
			if(tmp_c_ai_cnt > 0) {
				//corr_toMiddle = (-curr_track_width/2 + curr_toMiddle)/2;
				corr_toMiddle = -2.5 + this.getAiSideDist(curr_toMiddle, curr_aicars[tmp_c_ai[0]]);
				
				// Ʈ�� �ٱ��� ���
				if((-curr_track_width/2 + curr_toMiddle) > corr_toMiddle ) {
					corr_toMiddle = -curr_track_width/2  + curr_toMiddle - 1.0;
				}
				
				emer_turn_yn = 1.0;
				
			} else {
				
				corr_toMiddle = this.getKeepTrackSideDist(curr_toMiddle, curr_track_width);
			}
		
		// ���� ��� ������ �ִ� ���
		} else if (tmp_r_ai_cnt > 0 && tmp_l_ai_cnt > 0) {
			
			System.out.println("Left and Right ai car : " + tmp_l_ai_cnt + "," + tmp_r_ai_cnt + "," + tmp_c_ai_cnt);
			
			double tmp_left_width = 0.0;
			double tmp_right_width = 0.0;
			// ���� ������ �ִ� ���
			if(tmp_c_ai_cnt > 0) {
				
				// �¿�, ���� ��� ������ �ִ� ��� ��/�� �� ������ ū ������ �߰����� ����
				tmp_left_width = this.getAiSideDist(curr_aicars[tmp_l_ai[0]],curr_aicars[tmp_c_ai[0]]);
				tmp_right_width = this.getAiSideDist(curr_aicars[tmp_c_ai[0]],curr_aicars[tmp_r_ai[0]]);
				
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
							}
						}
					}
				}
				
				emer_turn_yn = 1.0;
		
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
				System.out.println("Forward ai car : " + tmp_c_ai_cnt);
				
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
				System.out.println("No ai car forward. Go Go!!!");
				System.out.println("track_curve_type : " + curr_track_curve_type);
				System.out.println("track_dist_straight : " + curr_track_dist_straight);
				System.out.println("track_whole_dist : " + curr_whole_track_dist_straight);
				
				// ��ȸ�� �ڽ�
				if(curr_track_curve_type == 1.0) {
					System.out.println("Right Curve " + curr_track_dist_straight + " forward.");
					
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
					System.out.println("Left Curve " + curr_track_dist_straight + " forward.");
					
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
		
		System.out.println("corr_toMiddle : " + corr_toMiddle);

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
		}else if (curr_track_dist_straight > 80 && curr_track_dist_straight <= 100) {
			curr_max_speed = 40;
		}else if (curr_track_dist_straight > 50 && curr_track_dist_straight <= 80) {
			curr_max_speed = 30;
		}else{
			curr_max_speed = 25; // 90�� �̻��϶� ���� �ӵ�(88~90km/h) 
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
		
		System.out.println("+++++++++++++++++ ���� �ӵ� ���[start] ++++++++++++++++++++++");
		System.out.println("curr_max_speed          ="+curr_max_speed);
		System.out.println("user_best_speed         ="+user_best_speed);
		System.out.println("curr_speed              ="+curr_speed + " m/s");
		System.out.println("curr_speed              ="+curr_speed*3.6 + " km/h");
		System.out.println("curr_angle              ="+curr_angle);
		System.out.println("curr_angle_abs          ="+curr_angle_abs + " ��");
		System.out.println("curr_track_dist_straight="+curr_track_dist_straight);
		System.out.println("+++++++++++++++++ ���� �ӵ� ���[end] ++++++++++++++++++++++");
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
			System.out.println("+++++++++++++++++ �극��ũ, ���� ���� �Լ�[start] ++++++++++++++++++++++");
			System.out.println("curr_speed               = "+curr_speed);
			System.out.println("curr_best_speed          = "+curr_best_speed);
			System.out.println("curr_track_dist_straight = "+curr_track_dist_straight);
			user_speed_ctl[0] = 0.1;
			
			if(curr_track_dist_straight < 20){
				user_speed_ctl[1] = 0.2;
			}else{
				user_speed_ctl[1] = 0.2;
			}
			System.out.println("user_brakeCtl="+user_speed_ctl[1]);
			System.out.println("+++++++++++++++++ �극��ũ, ���� ���� �Լ�[end] ++++++++++++++++++++++");
			
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
			System.out.println("+++++++++++++++++ �극��ũ, ���� ���� �Լ�2[start] ++++++++++++++++++++++");
			System.out.println("curr_speed               = "+curr_speed);
			System.out.println("curr_best_speed          = "+curr_best_speed);
			System.out.println("curr_track_dist_straight = "+curr_track_dist_straight);
			
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
			System.out.println("user_accelCtl="+user_speed_ctl[0]);
			System.out.println("user_brakeCtl="+user_speed_ctl[1]);
			System.out.println("+++++++++++++++++ �극��ũ, ���� ���� �Լ�2[end] ++++++++++++++++++++++");
			
		//}else{
		//	user_speed_ctl[0] = 0.4;
		//}		

		
		return user_speed_ctl;		
	}
	
			
}