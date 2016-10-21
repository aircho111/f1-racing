public class DrivingController {	
	public class DrivingCmd{
		public double steer;
		public double accel;
		public double brake;
		public int backward;
	};
	
	public double track_late_angle = 0.0;  // 이전 트랙 angle 보관
	
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
		
		// 직전 트랙Angle과 현재 트랙Angle과의 차이로 커브의 수준 파악을 위함.
		track_curve_level = track_current_angle - track_late_angle;
		
		System.out.println("===================================================");
		System.out.println("toStart : " + toStart);
		System.out.println("toMiddle : " + toMiddle); // 음수이면 중앙에서 오른쪽, 양수이면 왼쪽
		System.out.println("angle : " + angle);
		System.out.println("speed : " + speed);  // 수치가 이상하게 들어옴...
		System.out.println("track_width : " + track_width);
		System.out.println("track_late_angle : " + track_late_angle);
		System.out.println("track_curr_angle : " + track_current_angle);
		System.out.println("track_curve_level : " + track_curve_level);
		System.out.println("track_dist_straight : " + track_dist_straight); 
		
		System.out.println("---------------------------------------------------");
		
		// 차량이 이동할 트랙의 상대위치 (+값은 오른쪽, -값은 왼쪽), 장애물로 인해 이동이 불가한 경우 -100 리턴
		//corr_toMiddle = this.getCorrToMiddle(dist_cars, toMiddle, speed, angle, track_width, track_dist_straight, track_curve_type);
		corr_route[0] = 0.0;
		corr_route[1] = 0.0;
		corr_route = this.getCorrToMiddle(dist_cars, toMiddle, speed, angle, track_width, track_dist_straight, track_curve_type);
		
		emer_turn_yn = corr_route[0];   // 장애물 피하기 위한 경로 조정인 경우 0보다 큰값
		corr_toMiddle = corr_route[1];
		
		/* --- 가속/감속 추가 함수 필요 : 진희책임 -- */		
		// 차량의 break 조건 처리(임시로 전방 커브 10M 전에 차량의 속도가 110K 이상인 경우 브레이킹...속도값을 이상하게 주고있어 일단 시뮬레이터에서 주는 값 기준으로 계산함)
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
		
		// 이동할 트랙의 상대위치 리턴값이 -100인경우는 장애물로 인해 이동이 불가한 경우이므로 차량 속도를 줄임
		if(corr_toMiddle == -100.0) {
			corr_break = 0.3;
			//corr_accel = 0.0;
			//corr_toMiddle = 0.0;
			corr_toMiddle = this.getKeepTrackSideDist(toMiddle, track_width);
		}
		/*-----------------------------------------*/
		
		/* --- 트랙조건에 따른 angle계수 추가 함수 필요 : 우열책임 -- */
		// angle값에 대한 계수 계산(속도, 트랙의 조건에 따라 계산)
		streer_coeff = this.getSteerCoeff(speed, track_dist_straight);
		
		if(emer_turn_yn > 0.0) {
			streer_coeff = 1.0;
		}
		
		System.out.println("emergency turn yn : " + emer_turn_yn);
		
		// 차량이 회전할 angle값 계산
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
	
	// 경로 찾기
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
		
		// 장애물 차량 배영로 부터 전방, 좌측, 우측 별 ai 차량 배열 생성
		for(int i=1 ; i<curr_aicars.length ; i+=2) {
			//
			// 내차 기준 장애차량과의 간격
			tmp_ai_dist = this.getAiSideDist(curr_toMiddle, curr_aicars[i]);
						
			if(curr_aicars[i-1] > -100.0 && curr_aicars[i-1] < 100.0) {
				System.out.println("AI Car #" + (i+1)/2 + " : " + curr_aicars[i-1] + ", " + tmp_ai_dist);
			}
			
			
			// 처음 출발시는 모두 0.0이므로 제외
			if(curr_speed == 0.0){
				continue;
			}
			
			// 내 차보다 10M 뒤 80M 앞에 있는 ai차량은 일단 제외
			if(curr_aicars[i-1] <= backward_dist_max || curr_aicars[i-1] >= forward_dist_max) {
				continue;
			}
			
			
			
			
			/*================ 내차의 오른쪽 왼쪽 전방 AI 차량 배열 생성 ====================*/
			// 내차의 전방 충돌 위치 ai 차량 수
			// 전방 4~50M, 좌측우측 9M폭 사이에 있는 차량은 전방 차량으로 간주
			if((curr_aicars[i-1] > forward_dist_min && curr_aicars[i-1] < forward_dist_max) 
					&& (tmp_ai_dist > -(my_car_width + 0.5) && tmp_ai_dist < (my_car_width + 0.5))) {
				tmp_c_ai[tmp_c_ai_cnt] = i;
				tmp_c_ai_cnt++;
				System.out.println("   --> 전방 ai 차량수 : " + tmp_c_ai_cnt);
			} else {
				
				// ai차량이 내 차보다 앞에 있는 경우
//				if(curr_aicars[i-1] > 0.0) {
					
					// 내 차의 왼쪽에 위치하는 ai차량을  거리순으로 array에 저장
					if(tmp_ai_dist < 0.0) {
						if(tmp_l_ai_cnt == 0) { // 첫번째 왼쪽 ai차량의 배열 인덱스
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
						
						System.out.println("   --> 좌측 ai 차량수 : " + tmp_l_ai_cnt);
					
					// 내 차의 오른쪽에 위치하는 ai차량을 거리순으로 array에 저장	
					} else if (tmp_ai_dist >= 0.0) {
						if(tmp_r_ai_cnt == 0) { // 첫번째 왼쪽 ai차량의 배열 인덱스
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
						
						System.out.println("   --> 우측 ai 차량수 : " + tmp_r_ai_cnt);
					}
					
				// ai차량이 내 차보다 뒤에 있는 경우	
//				} else {
					
					
//					if((curr_aicars[i-1] > -10.0 && curr_aicars[i-1] < 0.0) 
//							&& (curr_aicars[i] > -2.5 && curr_aicars[i] < 2.5)) {
//						tmp_b_ai[tmp_b_ai_cnt] = i;
//						tmp_b_ai_cnt++;
//						System.out.println("   --> 후방 ai 차량수 : " + tmp_b_ai_cnt);
//					} else {
					
					// 후방 좌측 차량인 경우
//						if(tmp_ai_dist < 0.0) {
//							if(tmp_l_ai_cnt == 0) { // 첫번째 왼쪽 ai차량의 배열 인덱스
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
//							System.out.println("   --> 후방 좌측 ai 차량수 : " + tmp_l_ai_cnt);
//						
//						// 내 차의 오른쪽에 위치하는 ai차량을 toMiddle 거리순으로 array에 저장	
//						} else if (tmp_ai_dist >= 0.0) {
//							if(tmp_r_ai_cnt == 0) { // 첫번째 왼쪽 ai차량의 배열 인덱스
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
//							System.out.println("   --> 후방 우측 ai 차량수 : " + tmp_r_ai_cnt);
//						}
////					}
//					
//				}
				
			}
			
		} /* for문 끝 */
		/*================ 내차의 오른쪽 왼쪽 바로 앞쪽 AI 차량 배열 생성 끝 ====================*/
		System.out.println("---------------------------------------------------");
		
		/*================ 경로 결정 ====================*/
		// ai 차량이 왼쪽에 있고 오른쪽에 없는 경우
		if(tmp_r_ai_cnt == 0 && tmp_l_ai_cnt > 0) {
			System.out.println("Left ai car : " + tmp_l_ai_cnt + "," + tmp_c_ai_cnt);
			
			// 전방에 ai 차량이 있는 경우 오른방향으로
			//if(tmp_c_ai_cnt > 0  || curr_aicars[tmp_l_ai[0]] < 3.0) {
			if(tmp_c_ai_cnt > 0) {
				//corr_toMiddle = (curr_track_width/2 + curr_toMiddle)/2;
				corr_toMiddle = 2.5 + this.getAiSideDist(curr_toMiddle, curr_aicars[tmp_c_ai[0]]);  // 전방 장애물차의 차폭을 2.5M라고 가정
				
				// 트랙 바깥으로 벗어나는 경우 트랙까지만 경로 셋팅...전방에 있는 장애물과 부딪힐 경우도 생각해야함...
				if((curr_track_width/2 + curr_toMiddle) < corr_toMiddle ) {
					corr_toMiddle = curr_track_width/2  + curr_toMiddle - 1.0;
				}
				
				emer_turn_yn = 1.0;
			} else {
				
				corr_toMiddle = this.getKeepTrackSideDist(curr_toMiddle, curr_track_width);
			}
		
	    // ai 차량이 오른쪽에 있고 왼쪽에 없는 경우
		} else if (tmp_r_ai_cnt > 0 && tmp_l_ai_cnt == 0) {
			System.out.println("Right ai car : " + tmp_r_ai_cnt + "," + tmp_c_ai_cnt);
			
			// 바로 앞에 ai 차량이 있는 경우 왼쪽방향으로
			//if(tmp_c_ai_cnt > 0 || curr_aicars[tmp_r_ai[0]] > -3.0) {
			if(tmp_c_ai_cnt > 0) {
				//corr_toMiddle = (-curr_track_width/2 + curr_toMiddle)/2;
				corr_toMiddle = -2.5 + this.getAiSideDist(curr_toMiddle, curr_aicars[tmp_c_ai[0]]);
				
				// 트랙 바깥인 경우
				if((-curr_track_width/2 + curr_toMiddle) > corr_toMiddle ) {
					corr_toMiddle = -curr_track_width/2  + curr_toMiddle + 1.0;
				}
				
				emer_turn_yn = 1.0;
				
			} else {
				
				corr_toMiddle = this.getKeepTrackSideDist(curr_toMiddle, curr_track_width);
			}
		
		// 양쪽 모두 차량이 있는 경우
		} else if (tmp_r_ai_cnt > 0 && tmp_l_ai_cnt > 0) {
			
			System.out.println("Left and Right ai car : " + tmp_l_ai_cnt + "," + tmp_r_ai_cnt + "," + tmp_c_ai_cnt);
			
			double tmp_left_width = 0.0;
			double tmp_right_width = 0.0;
			// 전방 차량이 있는 경우
			if(tmp_c_ai_cnt > 0) {
				
				// 좌우, 전방 모두 차량이 있는 경우 좌/우 중 간격이 큰 방향의 중간으로 진행
				tmp_left_width = this.getAiSideDist(curr_aicars[tmp_l_ai[0]],curr_aicars[tmp_c_ai[0]]);
				tmp_right_width = this.getAiSideDist(curr_aicars[tmp_c_ai[0]],curr_aicars[tmp_r_ai[0]]);
				
				if(tmp_left_width > tmp_right_width) {
					
					//간격이 내 차량이 지나가기에 충분한 경우만 진행
					if(tmp_left_width > my_car_width) {
						corr_toMiddle = this.getAiSideDist(curr_toMiddle,curr_aicars[tmp_l_ai[0]]) + tmp_left_width/2;
						
					//간격이 내 차량이 지나가기에 충분하지 않은 경우  양쪽 차량의 바깥쪽으로 진행(간격이 작은 쪽 우선 체크)  
					} else {
						// 차량이 바로 옆에 있는지 체크(바로 옆에 있음...충돌)
						if(curr_aicars[tmp_r_ai[0]-1] > my_car_length) {
							corr_toMiddle = this.getAiSideDist(curr_toMiddle,curr_aicars[tmp_r_ai[0]]) + my_car_width;
						} else if(curr_aicars[tmp_l_ai[0]-1] > my_car_length) {
							corr_toMiddle = this.getAiSideDist(curr_toMiddle,curr_aicars[tmp_l_ai[0]]) + my_car_width;
						} else {
							// 양쪽이 모두 차량으로 막혀 있는 경우 전방 차량이 7M 앞에 올때까지는 현재 진행경로 유지
							if(curr_aicars[tmp_c_ai[0]-1] > 7.0) {
								corr_toMiddle = this.getKeepTrackSideDist(curr_toMiddle, curr_track_width);
							// 전방, 양쪽이 모두 막힐때 브레이킹
							} else {
								
								corr_toMiddle = -100.0;
							}
						}
						
					}
				} else {
					//간격이 내 차량이 지나가기에 충분한 경우만 진행
					if(tmp_right_width > my_car_width) {
						corr_toMiddle = this.getAiSideDist(curr_toMiddle,curr_aicars[tmp_r_ai[0]]) - tmp_right_width/2;
						
					//간격이 내 차량이 지나가기에 충분하지 않은 경우  양쪽 차량의 바깥쪽으로 진행(간격이 작은 쪽 우선 체크)  	
					} else {
						// 차량이 바로 옆에 있는지 체크(바로 옆에 있음...충돌)
						if(curr_aicars[tmp_l_ai[0]-1] > my_car_length) {
							corr_toMiddle = this.getAiSideDist(curr_toMiddle,curr_aicars[tmp_l_ai[0]]) + my_car_width;
						} else if(curr_aicars[tmp_r_ai[0]-1] > my_car_length) {
							corr_toMiddle = this.getAiSideDist(curr_toMiddle,curr_aicars[tmp_r_ai[0]]) + my_car_width;
						} else {
							// 양쪽이 모두 차량으로 막혀 있는 경우 전방 차량이 7M 앞에 올때까지는 현재 진행경로 유지
							if(curr_aicars[tmp_c_ai[0]-1] > 7.0) {
								corr_toMiddle = this.getKeepTrackSideDist(curr_toMiddle, curr_track_width);
							// 전방, 양쪽이 모두 막힐때 브레이킹
							} else {
								corr_toMiddle = -100.0;
							}
						}
					}
				}
				
				emer_turn_yn = 1.0;
		
			// 전방에 장애물차량이 없는 경우 현재 경로 유지
			} else {
				
				//corr_toMiddle = this.getAiSideDist(curr_toMiddle,curr_aicars[tmp_l_ai[0]]) + this.getAiSideDist(curr_aicars[tmp_l_ai[0],curr_aicars[tmp_r_ai[0]])/2;
				corr_toMiddle = this.getKeepTrackSideDist(curr_toMiddle, curr_track_width);
			}
			
		// 양쪽에 ai차량이 없는 경우	
		} else {
			
			double tmp_fst_forward_width = 0.0;
			
			// 전방에 ai차량이 있는 경우
			if(tmp_c_ai_cnt > 0) {
				System.out.println("Forward ai car : " + tmp_c_ai_cnt);
				
				tmp_fst_forward_width = this.getAiSideDist(curr_toMiddle,curr_aicars[tmp_c_ai[0]]);
				
				//if ( tmp_fst_forward_width > -2.5 && tmp_fst_forward_width < 2.5) {
				
					if ( tmp_fst_forward_width < 0.0) { // 왼쪽에 ai차량이 있을때
					    //corr_toMiddle = (curr_track_width/2 + curr_toMiddle)/2;
						
						// 내 차의 진행 방향이 우측인 경우는 그대로 우측으로
						//if(curr_angle >= 0.0) {
							if(curr_angle < 0.0) {
								corr_toMiddle = tmp_fst_forward_width + my_car_width + 2.0;
							} else {
								corr_toMiddle = tmp_fst_forward_width + my_car_width + 1.0;
							}
							
							// 직선주로에서 트랙을 벗어나는 경우 반대방향으로
							if(curr_track_dist_straight > 0.0 && ((curr_track_width/2 + curr_toMiddle) < corr_toMiddle) ) {
								corr_toMiddle = tmp_fst_forward_width - my_car_width - 2.0;
								//corr_toMiddle = curr_toMiddle;
							}
							
							
						//} else {
						//	corr_toMiddle = tmp_fst_forward_width - my_car_width;
							
							// 트랙을 벗어나는 경우 반대방향으로
						//	if((-curr_track_width/2 + curr_toMiddle) > corr_toMiddle ) {
						//		corr_toMiddle = tmp_fst_forward_width + my_car_width;
								//corr_toMiddle = curr_toMiddle;
						//	}
						//}
						
						
					} else {
						// 내 차의 진행 방향이 좌측인 경우는 그대로 좌측으로
						//if(curr_angle <= 0.0) {
						//corr_toMiddle = (-curr_track_width/2 + curr_toMiddle)/2;
						
							if(curr_angle > 0.0) {
								corr_toMiddle = tmp_fst_forward_width - my_car_width - 2.0;
							} else {
								corr_toMiddle = tmp_fst_forward_width - my_car_width - 1.0;
							}

							// 직선주로에서 트랙을 벗어나는 경우 반대으로
							if(curr_track_dist_straight > 0.0 && ((-curr_track_width/2 + curr_toMiddle) > corr_toMiddle) ) {
								corr_toMiddle = tmp_fst_forward_width + my_car_width + 2.0;
								//corr_toMiddle = curr_toMiddle;
							}
						//} else {
							
						//	corr_toMiddle = tmp_fst_forward_width + my_car_width;
							
							// 트랙을 벗어나는 경우 반대방향으로
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
				
				// 우회전 코스
				if(curr_track_curve_type == 1.0) {
					System.out.println("Right Curve " + curr_track_dist_straight + " forward.");
					//전방 10M 전까지는 좌측으로 주행
					if(curr_track_dist_straight > 10.0) {
						if(curr_toMiddle > 0) {
							corr_toMiddle = (-(curr_track_width-2)/2 + curr_toMiddle)/5;
							
							// 트랙 바깥인 경우
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
							
							// 트랙 바깥으로 벗어나는 경우 트랙까지만 경로 셋팅...전방에 있는 장애물과 부딪힐 경우도 생각해야함...
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
	
	// 차량 간 Mid값으로 서로의 간격 구하기 (내차 기준)
	private double getAiSideDist (double curr_toMiddle, double curr_aiMiddle) {
		double ret_dist = 0.0;
		
		ret_dist = curr_toMiddle - curr_aiMiddle ;
		
		return ret_dist;
	}
	
	// Mid값으로 트랙 사이드까지 간격 구하기 (내차 기준)
	private double getTrackSideDist (double curr_toMiddle, double curr_track_width, int side) {
		double ret_dist = 0.0;
		
		if(side == 1) { // 오른쪽 사이드까지 거리
			ret_dist = curr_track_width/2 + curr_toMiddle;
		} else if(side == 2) { // 왼쪽 사이드까지 거리
			ret_dist = -(curr_track_width/2) + curr_toMiddle;
		}
		
		return ret_dist;
	}
	
	// 현재 진행 경로를 유지하기 위한 값
	private double getKeepTrackSideDist (double curr_toMiddle, double curr_track_width) {
		double ret_corr_toMiddle = 0.0;
		double tmp_r_track_side = this.getTrackSideDist(curr_toMiddle, curr_track_width, 1);
		double tmp_l_track_side = this.getTrackSideDist(curr_toMiddle, curr_track_width, 2);
		
		//전방에 장애물 차량이 없는 경우는 그냥 가던 길로
		
		if(tmp_r_track_side < 0.0 || tmp_l_track_side > 0.0) {
			ret_corr_toMiddle = tmp_r_track_side;
		} 
		
		return ret_corr_toMiddle;
	}
			
}