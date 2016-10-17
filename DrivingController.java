public class DrivingController {	
	public class DrivingCmd{
		public double steer;
		public double accel;
		public double brake;
		public int backward;
	};

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
		double streer_coeff = 1.0;
		double corr_toMiddle = 0.0;
		System.out.println("toMiddle : " + toMiddle);
		System.out.println("angle : " + angle);
		System.out.println("speed : " + speed);
		System.out.println("track_width : " + track_width);
		
		//steer_angle = streer_coeff * (angle - toMiddle/track_width);
		corr_toMiddle = this.getCorrToMiddle(dist_cars, toMiddle, track_width, track_dist_straight, track_curve_type);
		steer_angle = this.getSteerAngle(angle, corr_toMiddle, track_width, streer_coeff);
		System.out.println("steer_angle : " + steer_angle);
		////////////////////// output values		
		cmd.steer = steer_angle;
		cmd.accel = 0.2;
		cmd.brake = 0.0;
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
	
	private double getCorrToMiddle(double[] curr_aicars, double curr_toMiddle, double curr_track_width, double curr_track_dist_straight, double curr_track_curve_type) {
		double corr_toMiddle = 0.0;
		double tmp_ai_dist = 0.0;
		double tmp_ai_toMiddle = 0.0;
		
		int[] tmp_r_ai = new int[5];
		int[] tmp_l_ai = new int[5];
		int[] tmp_c_ai = new int[5];
		int tmp_r_ai_cnt = 0;
		int tmp_r_curr_idx = 0;
		int tmp_l_ai_cnt = 0;
		int tmp_l_curr_idx = 0;
		int tmp_c_ai_cnt = 0;
		
		for(int i=0 ; i<5 ; i++){
			tmp_r_ai[i] = -1;
			tmp_l_ai[i] = -1;
			tmp_c_ai[i] = -1;
		}
		
		for(int i=1 ; i<curr_aicars.length ; i+=2) {
			//
			System.out.println("AI Car #" + (i+1)/2 + " : " + curr_aicars[i-1] + ", " + curr_aicars[i]);
			// 내 차보다 뒤에 있는 ai차량은 일단 제외
			if(curr_aicars[i-1] < 0) {
				continue;
			}
			
			// 내 차의 왼쪽에 위치하는 ai차량을 toMiddle 거리순으로 array에 저장
			if(curr_aicars[i] < 0) {
				if(tmp_l_ai_cnt == 0) { // 첫번째 왼쪽 ai차량의 배열 인덱스
					tmp_l_ai[0] = i;
				} else {
					
					for(int j=0 ; j<5 ; j++) {
						if(tmp_l_ai[j] < 0) {
							tmp_l_ai[j] = i;
							break;
						} else {
							
							if(curr_aicars[tmp_l_ai[j]] < curr_aicars[i]) {
								for(int k=j ; k < tmp_l_ai_cnt ; k++) {
									tmp_l_ai[k+1] = tmp_l_ai[k];
								}
								
								tmp_l_ai[j] = i;
							}
						}
						
						
					}
				}
				
				tmp_l_ai_cnt++;
			
			// 내 차의 오른쪽에 위치하는 ai차량을 toMiddle 거리순으로 array에 저장	
			} else if (curr_aicars[i] > 0) {
				if(tmp_r_ai_cnt == 0) { // 첫번째 왼쪽 ai차량의 배열 인덱스
					tmp_r_ai[0] = i;
				} else {
					
					for(int j=0 ; j<5 ; j++) {
						if(tmp_r_ai[j] < 0) {
							tmp_r_ai[j] = i;
							break;
						} else {
							
							if(curr_aicars[tmp_r_ai[j]] < curr_aicars[i]) {
								for(int k=j ; k < tmp_r_ai_cnt ; k++) {
									tmp_r_ai[k+1] = tmp_r_ai[k];
								}
								
								tmp_r_ai[j] = i;
							}
						}
						
						
					}
				}
				
				tmp_r_ai_cnt++;
				
			} else {
				tmp_c_ai[tmp_c_ai_cnt] = i;
				tmp_c_ai_cnt++;
			}
			
		}
		
		// 앞쪽에 우회전 코스가 있으면 우측 우선
		//if(curr_track_curve_type == 1) {
			
			// 오른쪽에 ai차량이 없는 경우
			//if(tmp_r_ai_cnt == 0) {
			//	corr_toMiddle = curr_track_width/2 + curr_toMiddle;
			//} else {
			//	
			//}
			
		//}
		
		// 오른쪽에 ai 차량이 없는 경우
		if(tmp_r_ai_cnt == 0 && tmp_l_ai_cnt > 0) {
			corr_toMiddle = (curr_track_width/2 + curr_toMiddle)/2;
		
	    // 왼쪽에 ai 차량이 없는 경우
		} else if (tmp_r_ai_cnt > 0 && tmp_l_ai_cnt == 0) {
			corr_toMiddle = (-curr_track_width/2 + curr_toMiddle)/2;
		
		// 양쪽 모두 차량이 있는 경우 중간으로 ... 두 차량 사이가 좁은 경우 (2M 이하)는 거리가 좀 더 떨어진 쪽으로
		} else if (tmp_r_ai_cnt > 0 && tmp_l_ai_cnt > 0) {
			
			if(curr_aicars[tmp_r_ai[0]] - curr_aicars[tmp_l_ai[0]] > 2.0) {
				corr_toMiddle = curr_aicars[tmp_l_ai[0]] + (curr_aicars[tmp_r_ai[0]] - curr_aicars[tmp_l_ai[0]])/2;
			} else {
				if(curr_aicars[tmp_r_ai[0]-1] > curr_aicars[tmp_l_ai[0]-1]) {
					corr_toMiddle = curr_aicars[tmp_r_ai[0]] ;
				} else {
					corr_toMiddle = curr_aicars[tmp_l_ai[0]] ;
				}
			}
		} else {
			if(tmp_c_ai_cnt > 0) {
				if (curr_toMiddle > 0) {
					corr_toMiddle = (curr_track_width/2 + curr_toMiddle)/2;
				} else {
					corr_toMiddle = (-curr_track_width/2 + curr_toMiddle)/2;
				}
			} else {
				corr_toMiddle = curr_toMiddle;
			}
		}
		
		System.out.println("corr_toMiddle : " + corr_toMiddle);
		
		return corr_toMiddle;
	}
}
