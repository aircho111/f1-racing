public class DrivingController {	
	public class DrivingCmd{
		public double steer;
		public double accel;
		public double brake;
		public int backward;
	};
	
	public double track_late_angle = 0.0;  // ���� Ʈ�� angle ����
	
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
		double emer_turn_yn = -1.0;
		double[] corr_route = new double[2]; 
		double corr_break = 0.0;
		double corr_accel = 0.25;
		double track_curve_level = 0.0;
		
		// ���� Ʈ��Angle�� ���� Ʈ��Angle���� ���̷� Ŀ���� ���� �ľ��� ����.
		track_curve_level = track_current_angle - track_late_angle;
		
		System.out.println("===================================================");
		System.out.println("toStart : " + toStart);
		System.out.println("toMiddle : " + toMiddle); // �����̸� �߾ӿ��� ������, ����̸� ����
		System.out.println("angle : " + angle);
		System.out.println("speed : " + speed);  // ��ġ�� �̻��ϰ� ����...
		System.out.println("track_width : " + track_width);
		System.out.println("track_late_angle : " + track_late_angle);
		System.out.println("track_curr_angle : " + track_current_angle);
		System.out.println("track_curve_level : " + track_curve_level);
		System.out.println("track_dist_straight : " + track_dist_straight); 
		
		System.out.println("---------------------------------------------------");
		
		// ������ �̵��� Ʈ���� �����ġ (+���� ������, -���� ����), ��ֹ��� ���� �̵��� �Ұ��� ��� -100 ����
		//corr_toMiddle = this.getCorrToMiddle(dist_cars, toMiddle, speed, angle, track_width, track_dist_straight, track_curve_type);
		corr_route[0] = 0.0;
		corr_route[1] = 0.0;
		corr_route = this.getCorrToMiddle(dist_cars, toMiddle, speed, angle, track_width, track_dist_straight, track_curve_type);
		
		emer_turn_yn = corr_route[0];   // ��ֹ� ���ϱ� ���� ��� ������ ��� 0���� ū��
		corr_toMiddle = corr_route[1];
		
		/* --- ����/���� �߰� �Լ� �ʿ� : ����å�� -- */		
		// ������ break ���� ó��(�ӽ÷� ���� Ŀ�� 10M ���� ������ �ӵ��� 110K �̻��� ��� �극��ŷ...�ӵ����� �̻��ϰ� �ְ��־� �ϴ� �ùķ����Ϳ��� �ִ� �� �������� �����)
		if(track_dist_straight < 10.0){
			if( speed > 30.0) {
				corr_break = 0.2;
				corr_accel = 0.1;
			} else if ( speed > 23 && speed <= 30.0){
				
				if(track_curve_level < -0.045 || track_curve_level > 0.045) {
					corr_break = 0.2;
					corr_accel = 0.1;
				} else {
					corr_break = 0.0;
					corr_accel = 0.1;
				}
			} else {
				corr_break = 0.0;
				corr_accel = 0.2;				
			}
		} else {
			if( speed > 30.0) {
				corr_break = 0.0;
				corr_accel = 0.2;
			} else if ( speed > 23 && speed <= 30.0){
				corr_break = 0.0;
				corr_accel = 0.25;
			} else {
				corr_break = 0.0;
				corr_accel = 0.3;
			}
		}
		
		// �̵��� Ʈ���� �����ġ ���ϰ��� -100�ΰ��� ��ֹ��� ���� �̵��� �Ұ��� ����̹Ƿ� ���� �ӵ��� ����
		if(corr_toMiddle == -100.0) {
			corr_break = 0.3;
			//corr_accel = 0.0;
			//corr_toMiddle = 0.0;
			corr_toMiddle = this.getKeepTrackSideDist(toMiddle, track_width);
		}
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
		
		this.track_late_angle = track_current_angle;
		/*-----------------------------------------*/
		
		////////////////////// output values		
		cmd.steer = steer_angle;
		cmd.accel = corr_accel;
		cmd.brake = corr_break;
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
	
	private double getSteerAngle(double curr_angle, double curr_toMiddle, double curr_track_width, double streer_coeff) {
		double steer_angle = 0.0;
		
		steer_angle = streer_coeff * (curr_angle - curr_toMiddle/curr_track_width);
		
		return steer_angle;
	}
	
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
	
	// ��� ã��
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
		
		double[] ret_corr_route = new double[2];
		
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
				System.out.println("   --> ���� ai ������ : " + tmp_c_ai_cnt);
			} else {
				
				// ai������ �� ������ �տ� �ִ� ���
//				if(curr_aicars[i-1] > 0.0) {
					
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
						
						System.out.println("   --> ���� ai ������ : " + tmp_l_ai_cnt);
					
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
						
						System.out.println("   --> ���� ai ������ : " + tmp_r_ai_cnt);
					}
					
				// ai������ �� ������ �ڿ� �ִ� ���	
//				} else {
					
					
//					if((curr_aicars[i-1] > -10.0 && curr_aicars[i-1] < 0.0) 
//							&& (curr_aicars[i] > -2.5 && curr_aicars[i] < 2.5)) {
//						tmp_b_ai[tmp_b_ai_cnt] = i;
//						tmp_b_ai_cnt++;
//						System.out.println("   --> �Ĺ� ai ������ : " + tmp_b_ai_cnt);
//					} else {
					
					// �Ĺ� ���� ������ ���
//						if(tmp_ai_dist < 0.0) {
//							if(tmp_l_ai_cnt == 0) { // ù��° ���� ai������ �迭 �ε���
//								tmp_l_ai[0] = i;
//							} else {
//								
//								for(int j=0 ; j<5 ; j++) {
//									if(tmp_l_ai[j] < 0) {
//										tmp_l_ai[j] = i;
//										break;
//									} else {
//										
//										tmp_pre_ai_dist = this.getAiSideDist(curr_toMiddle, curr_aicars[tmp_l_ai[j]]);
//										
//										if(tmp_pre_ai_dist < tmp_ai_dist) {
//											for(int k=j ; k < tmp_l_ai_cnt ; k++) {
//												tmp_l_ai[k+1] = tmp_l_ai[k];
//											}
//											
//											tmp_l_ai[j] = i;
//										}
//									}
//									
//									
//								}
//							}
//							
//							tmp_l_ai_cnt++;
//							
//							System.out.println("   --> �Ĺ� ���� ai ������ : " + tmp_l_ai_cnt);
//						
//						// �� ���� �����ʿ� ��ġ�ϴ� ai������ toMiddle �Ÿ������� array�� ����	
//						} else if (tmp_ai_dist >= 0.0) {
//							if(tmp_r_ai_cnt == 0) { // ù��° ���� ai������ �迭 �ε���
//								tmp_r_ai[0] = i;
//							} else {
//								
//								for(int j=0 ; j<5 ; j++) {
//									if(tmp_r_ai[j] < 0) {
//										tmp_r_ai[j] = i;
//										break;
//									} else {
//										
//										tmp_pre_ai_dist = this.getAiSideDist(curr_toMiddle, curr_aicars[tmp_r_ai[j]]);
//										
//										if(tmp_pre_ai_dist < tmp_ai_dist) {
//											for(int k=j ; k < tmp_r_ai_cnt ; k++) {
//												tmp_r_ai[k+1] = tmp_r_ai[k];
//											}
//											
//											tmp_r_ai[j] = i;
//										}
//									}
//									
//								}
//							}
//							
//							tmp_r_ai_cnt++;
//							
//							System.out.println("   --> �Ĺ� ���� ai ������ : " + tmp_r_ai_cnt);
//						}
////					}
//					
//				}
				
			}
			
		} /* for�� �� */
		/*================ ������ ������ ���� �ٷ� ���� AI ���� �迭 ���� �� ====================*/
		System.out.println("---------------------------------------------------");
		
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
					corr_toMiddle = curr_track_width/2  + curr_toMiddle - 1.0;
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
					corr_toMiddle = -curr_track_width/2  + curr_toMiddle + 1.0;
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
				
				//if ( tmp_fst_forward_width > -2.5 && tmp_fst_forward_width < 2.5) {
				
					if ( tmp_fst_forward_width < 0.0) { // ���ʿ� ai������ ������
					    //corr_toMiddle = (curr_track_width/2 + curr_toMiddle)/2;
						
						// �� ���� ���� ������ ������ ���� �״�� ��������
						//if(curr_angle >= 0.0) {
							if(curr_angle < 0.0) {
								corr_toMiddle = tmp_fst_forward_width + my_car_width + 2.0;
							} else {
								corr_toMiddle = tmp_fst_forward_width + my_car_width + 1.0;
							}
							
							// �����ַο��� Ʈ���� ����� ��� �ݴ��������
							if(curr_track_dist_straight > 0.0 && ((curr_track_width/2 + curr_toMiddle) < corr_toMiddle) ) {
								corr_toMiddle = tmp_fst_forward_width - my_car_width - 2.0;
								//corr_toMiddle = curr_toMiddle;
							}
							
							
						//} else {
						//	corr_toMiddle = tmp_fst_forward_width - my_car_width;
							
							// Ʈ���� ����� ��� �ݴ��������
						//	if((-curr_track_width/2 + curr_toMiddle) > corr_toMiddle ) {
						//		corr_toMiddle = tmp_fst_forward_width + my_car_width;
								//corr_toMiddle = curr_toMiddle;
						//	}
						//}
						
						
					} else {
						// �� ���� ���� ������ ������ ���� �״�� ��������
						//if(curr_angle <= 0.0) {
						//corr_toMiddle = (-curr_track_width/2 + curr_toMiddle)/2;
						
							if(curr_angle > 0.0) {
								corr_toMiddle = tmp_fst_forward_width - my_car_width - 2.0;
							} else {
								corr_toMiddle = tmp_fst_forward_width - my_car_width - 1.0;
							}

							// �����ַο��� Ʈ���� ����� ��� �ݴ�����
							if(curr_track_dist_straight > 0.0 && ((-curr_track_width/2 + curr_toMiddle) > corr_toMiddle) ) {
								corr_toMiddle = tmp_fst_forward_width + my_car_width + 2.0;
								//corr_toMiddle = curr_toMiddle;
							}
						//} else {
							
						//	corr_toMiddle = tmp_fst_forward_width + my_car_width;
							
							// Ʈ���� ����� ��� �ݴ��������
						//	if((curr_track_width/2 + curr_toMiddle) < corr_toMiddle ) {
						//		corr_toMiddle = tmp_fst_forward_width - my_car_width;
								//corr_toMiddle = curr_toMiddle;
						//	}
							
						//}
					}
					
					emer_turn_yn = 1.0;

			} else {
				System.out.println("No ai car forward. Go Go!!!");
				System.out.println("track_curve_type : " + curr_track_curve_type);
				System.out.println("track_dist_straight : " + curr_track_dist_straight);
				
				// ��ȸ�� �ڽ�
				if(curr_track_curve_type == 1.0) {
					System.out.println("Right Curve " + curr_track_dist_straight + " forward.");
					//���� 10M �������� �������� ����
					if(curr_track_dist_straight > 10.0) {
						if(curr_toMiddle > 0) {
							corr_toMiddle = (-(curr_track_width-2)/2 + curr_toMiddle)/5;
							
							// Ʈ�� �ٱ��� ���
							if((-curr_track_width/2 + 1.5 + curr_toMiddle) > corr_toMiddle ) {
								corr_toMiddle = -curr_track_width/2  + curr_toMiddle + 1.0;
							}
							
						} else {
							corr_toMiddle = curr_toMiddle/5;
						}
					} else {
						//if(curr_track_dist_straight == 0.0) {
						//	corr_toMiddle = 0.0;
						//} else {
							corr_toMiddle = ((curr_track_width-1)/2 + curr_toMiddle)/3;
						//}
					}
					
				} else if(curr_track_curve_type == 2.0){
					System.out.println("Left Curve " + curr_track_dist_straight + " forward.");
					
					if(curr_track_dist_straight > 10.0) {
						
						if(curr_toMiddle < 0) {
							corr_toMiddle = ((curr_track_width-2)/2 + curr_toMiddle)/5;
							
							// Ʈ�� �ٱ����� ����� ��� Ʈ�������� ��� ����...���濡 �ִ� ��ֹ��� �ε��� ��쵵 �����ؾ���...
							if((curr_track_width/2 - 1.5 + curr_toMiddle) < corr_toMiddle ) {
								corr_toMiddle = curr_track_width/2  + curr_toMiddle - 1.0;
							}
							
						} else {
							corr_toMiddle = curr_toMiddle/5;
						}
						
					} else {
						//if(curr_track_dist_straight == 0.0) {
						//	corr_toMiddle = 0.0;
						//} else {
							corr_toMiddle = (-(curr_track_width-1)/2 + curr_toMiddle)/3 ;
						//}
					}
				} else {
				
					//corr_toMiddle = curr_toMiddle;
					corr_toMiddle = this.getKeepTrackSideDist(curr_toMiddle, curr_track_width);
				}
			}
		}

		
		ret_corr_route[0] = emer_turn_yn;
		ret_corr_route[1] = corr_toMiddle;
		
		System.out.println("corr_toMiddle : " + corr_toMiddle);

		return ret_corr_route;
	}
	
	// ���� �� Mid������ ������ ���� ���ϱ� (���� ����)
	private double getAiSideDist (double curr_toMiddle, double curr_aiMiddle) {
		double ret_dist = 0.0;
		
		ret_dist = curr_toMiddle - curr_aiMiddle ;
		
		return ret_dist;
	}
	
	// Mid������ Ʈ�� ���̵���� ���� ���ϱ� (���� ����)
	private double getTrackSideDist (double curr_toMiddle, double curr_track_width, int side) {
		double ret_dist = 0.0;
		
		if(side == 1) { // ������ ���̵���� �Ÿ�
			ret_dist = curr_track_width/2 + curr_toMiddle;
		} else if(side == 2) { // ���� ���̵���� �Ÿ�
			ret_dist = -(curr_track_width/2) + curr_toMiddle;
		}
		
		return ret_dist;
	}
	
	// ���� ���� ��θ� �����ϱ� ���� ��
	private double getKeepTrackSideDist (double curr_toMiddle, double curr_track_width) {
		double ret_corr_toMiddle = 0.0;
		double tmp_r_track_side = this.getTrackSideDist(curr_toMiddle, curr_track_width, 1);
		double tmp_l_track_side = this.getTrackSideDist(curr_toMiddle, curr_track_width, 2);
		
		//���濡 ��ֹ� ������ ���� ���� �׳� ���� ���
		
		if(tmp_r_track_side < 0.0 || tmp_l_track_side > 0.0) {
			ret_corr_toMiddle = tmp_r_track_side;
		} 
		
		return ret_corr_toMiddle;
	}
			
}