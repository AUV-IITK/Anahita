#include <cruc.h>

Cruc::Cruc() {
	this->loadParams();
	this->front_roi_pub = it.advertise("/anahita/roi", 1);
    std::string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};
    std::string trackerType = trackerTypes[2];
 
    if (trackerType == "KCF") {
        tracker1 = cv::TrackerKCF::create();
    }
    else if (trackerType == "MIL") {
        tracker1 = cv::TrackerMIL::create();
    }
}


void Cruc::loadParams() {
	nh.getParam("/anahita/vision/cruc/b_min", front_low_b_);
	nh.getParam("/anahita/vision/cruc/b_max", front_high_b_);
	nh.getParam("/anahita/vision/cruc/g_min", front_low_g_);
	nh.getParam("/anahita/vision/cruc/g_max", front_high_g_);
	nh.getParam("/anahita/vision/cruc/r_min", front_low_r_);
	nh.getParam("/anahita/vision/cruc/r_max", front_high_r_);
	nh.getParam("/anahita/vision/cruc/closing_mat_point", front_closing_mat_point_);
	nh.getParam("/anahita/vision/cruc/closing_iter", front_closing_iter_);
	nh.getParam("/anahita/vision/cruc/opening_mat_point", front_opening_mat_point_);
	nh.getParam("/anahita/vision/cruc/opening_iter", front_opening_iter_);
	nh.getParam("/anahita/vision/cruc/bilateral_iter", front_bilateral_iter_);
}

void Cruc::updateCoordinates (cv::Point points) {
        TL.x = points.x;
        TL.y = points.y;

        TL_init = true;
}

 void Cruc::InitTracker (cv::Mat& src_img, cv::Mat& thres_img) {
     std::vector<std::vector<cv::Point> > contours;
     std::vector<cv::Vec4i> hierarchy;
     cv::findContours (thres_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
     if(contours.size()>1)
     { bbox1 = boundingRect(cv::Mat(contours[0]));

       rectangle(src_img, bbox1, cv::Scalar(255, 0, 0), 2, 1);

       tracker1->init(src_img, bbox1);
       initTracker = true;}
 }
void Cruc::updateTracker (cv::Mat& src_img, int& count1) {
    if (tracker1->update (src_img, bbox1)){
        rectangle(src_img, bbox1, cv::Scalar(255, 0, 0), 2, 1);
	cv::circle(src_img, cv::Point(int(bbox1.x+bbox1.width/2), int(bbox1.y+bbox1.height/2)), 5, cv::Scalar(0, 255, 0), 5, 8, 0);
        updateCoordinates (cv::Point(int(bbox1.x+bbox1.width/2), int(bbox1.y+bbox1.height/2)));
       // std::cout<<"bbox"<<std::endl;
       // std::cout<<bbox.x<<std::endl;
        count1=0;
        countn=0;
        }
    else{ 
        ROS_ERROR ("BBox1 failing....");
        count1= count1+1;
        //if(count>3){
        //    std::cout<<"bbox"<<std::endl;
        //    InitTracker (src_img, mask1);
        //    }
        }
}

void Cruc::spinThreadFront()
{
	cv::Mat temp_src;
	cv::Mat marked_img;
    cv::Mat canny_edge;
    cv::Mat blu;
    cv::Mat blur1;
    cv::Mat mask;
    cv::Mat hsv;
    cv::Mat edges;
    cv::Mat frame;

	sensor_msgs::ImagePtr front_image_marked_msg;
	sensor_msgs::ImagePtr front_image_thresholded_msg;
	ros::Rate loop_rate(15);

    double arr[10][2];
    double arr1[5];
    double great=0;
    
	for(int i=0; i<10; i++)
	{ for(int j=0; j<2; j++)
	 { arr[i][j]= 0.0;}
        }
        for(int i=0; i<5; i++)
	{
	  arr1[i]= 0.0;
        }
	int count = 0;
        int countbox=0;
	int l=10;
	size_t num = 160;
	while (ros::ok())
	{
		if (close_task) {
			close_task = false;
			break;
		}

		if (!image_front.empty()) {
                        vision_mutex.lock();
			temp_src = image_front.clone();
			marked_img = image_front.clone();
                        vision_mutex.unlock();
            cv::bilateralFilter(image_front,blu, 9, 75, 75);
            cv::GaussianBlur(blu,blur1, cv::Size(15, 15), 0);
            cv::cvtColor(blur1,hsv, cv::COLOR_BGR2HSV);
            cv::inRange(hsv, cv::Scalar(80,100,100), cv::Scalar(91, 245, 240), mask);
            cv::Canny(mask,edges, 50, 150,3);

                        
            std::vector<std::vector<cv::Point> > contours;
            std::vector<cv::Vec4i> hierarchy;
	    cv::findContours (mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
            
            great = 0;
            if(contours.size()>0){
            cv::Rect2d bbox2 = boundingRect(cv::Mat(contours[0]));
            rectangle(image_front, bbox2, cv::Scalar(0, 255, 0), 2, 1);
            std::cout<<bbox2.height<<std::endl;
            
            if(bbox2.height>30){
            arr1[countbox] = bbox2.height;
            countbox=countbox+1;
            if( countbox>4) countbox= countbox-5;
            }
            for(int i=0; i<5; i++)
            {
             if(great<arr1[i])
             great = arr1[i];
             }
            if(great>200)
            std::cout<<"HOUGHLINES"<<std::endl;
            }
            //if(!initTracker){
	    //cv::Rect2d bbox = boundingRect(cv::Mat(contours[0]));
            //cv::Rect2d bbox(200, 23, 150, 320);
            //rectangle(image_front, bbox, cv::Scalar(255, 0, 0), 2, 1);
           // std::cout<<contours[0]<<std::endl;
	    //tracker1->init(image_front, bbox);
            //initTracker = true;}


            std::vector<cv::Vec2f> lines; 
	    if(!initTracker)  InitTracker (image_front, mask);
	   // else if(initTracker )  updateTracker (image_front, countn);
            
	    if(countn>10) 
            { initTracker = false;  delete tracker1 ;  countn=0;}
          //  std::cout<<countn<<std::endl;
          //  if(contours.size()==0)
          //  { initTracker = false;  }
	    if(num>=150) l=30;
	    else if(num>=120 && num< 150) l = 20;
            else if (num>=90 && num <120) l=13;
	    else if (num>0 && num <90) l=12;
            cv::HoughLines(edges,lines, 1, CV_PI / 180, l);
	    num = lines.size();
	//    std::cout<<num<<std::endl;
            
        int nov=0,noh=0;
        int countv=0,counth=0,counti=0;
        float meanx=0,meany=0;
        float x=0,y=0,xm=0,ym=0;
         
	for(size_t i=0;i<lines.size();i++){
            float a = cos(lines[i][1]);
            float b = sin(lines[i][1]);
            float x0 = a*lines[i][0];
            float y0 = b*lines[i][0];
            float x1 = x0 + 700*(-b);
            float y1 = y0 + 700*(a);
            float x2 = x0 - 700*(-b);
            float y2 = y0 - 700*(a);
            
            if(abs(x1-x2)<80) nov= nov+1;
            else if(abs(y1-y2)<80) noh = noh+1;
                    }
    
        float arrv[nov][4];
        float arrh[noh][4];
        int inter = noh*nov;
        float arri[inter][2];
   
        for(size_t i=0;i<lines.size();i++){
            float a = cos(lines[i][1]);
            float b = sin(lines[i][1]);
            float x0 = a*lines[i][0];
            float y0 = b*lines[i][0];
            float x1 = x0 + 700*(-b);
            float y1 = y0 + 700*(a);
            float x2 = x0 - 700*(-b);
            float y2 = y0 - 700*(a);

            
            if(abs(x1-x2)<80){
                arrv[countv][0] = x1;
                arrv[countv][1] = y1;                
                arrv[countv][2] = x2;                
                arrv[countv][3] = y2;
                countv = countv+1;
                }    

                if(abs(y1-y2)<80){
                arrh[counth][0] = x1;
                arrh[counth][1] = y1;                
                arrh[counth][2] = x2;                
                arrh[counth][3] = y2;
                counth = counth +1;
                }

           // cv::line(image_front,cv::Point(x1,y1),cv::Point(x2,y2),cv::Scalar(0,0,255),2, 8, 0);
            }

   for(int i=0;i<countv;i++){
       for(int j=0;j<counth;j++){
                float div = (arrv[i][0]-arrv[i][2])*(arrh[j][1]-arrh[j][3])-(arrv[i][1]-arrv[i][3])*(arrh[j][0]-arrh[j][2]);

                if(div!=0){
                    arri[counti][0] =( (arrv[i][0]*arrv[i][3] - arrv[i][1]*arrv[i][2])*(arrh[j][0]-arrh[j][2]) - (arrv[i][0]-arrv[i][2])*(arrh[j][0]*arrh[j][3]-arrh[j][1] * arrh[j][2]))/((arrv[i][0]-arrv[i][2])*(arrh[j][1]-arrh[j][3])-(arrv[i][1]-arrv[i][3])*(arrh[j][0]-arrh[j][2]));
                    arri[counti][1] =( (arrv[i][0]*arrv[i][3] - arrv[i][1]*arrv[i][2])*(arrh[j][1]-arrh[j][3]) - (arrv[i][1]-arrv[i][3])*(arrh[j][0]*arrh[j][3]-arrh[j][1] * arrh[j][2]))/((arrv[i][0]-arrv[i][2])*(arrh[j][1]-arrh[j][3])-(arrv[i][1]-arrv[i][3])*(arrh[j][0]-arrh[j][2])) ;
                counti=counti+1;
                }
        }
    }


    for(int i=0; i<counti;i++){
            meanx = meanx + arri[i][0];
            meany = meany + arri[i][1];
    }

    if(counti!=0){
            x= (1.0 * meanx)/counti;
            y= (1.0 * meany)/counti;
    }

    if(x!=0){
            arr[count][0] = x;
            arr[count][1] = y;
            count=count+1;
            if( count>9) count= count-10;
    }

    if(x!=0){    
            front_x_coordinate.data = x;
			front_y_coordinate.data = y;
			front_z_coordinate.data = 0;

			front_x_coordinate_pub.publish(front_x_coordinate);
			front_y_coordinate_pub.publish(front_y_coordinate);
			front_z_coordinate_pub.publish(front_z_coordinate);
            front_image_marked_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_front).toImageMsg();
        	front_marked_pub.publish(front_image_marked_msg);

			front_image_thresholded_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", edges).toImageMsg();
        	front_thresholded_pub.publish(front_image_thresholded_msg);
            cv::circle(image_front, cv::Point(int(x), int(y)), 5, cv::Scalar(255, 0, 0), 5, 8, 0);
    }
    else{
            for(int i=0; i<10; i++){
                xm = xm+arr[i][0];
                ym = ym+arr[i][1];
                }
            front_x_coordinate.data = xm/10;
			front_y_coordinate.data = ym/10;
			front_z_coordinate.data = 0;

			front_x_coordinate_pub.publish(front_x_coordinate);
			front_y_coordinate_pub.publish(front_y_coordinate);
			front_z_coordinate_pub.publish(front_z_coordinate);
            front_image_marked_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_front).toImageMsg();
        	front_marked_pub.publish(front_image_marked_msg);

			front_image_thresholded_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", edges).toImageMsg();
        	front_thresholded_pub.publish(front_image_thresholded_msg);
            cv::circle(image_front,  cv::Point((int)(xm/10), (int)(ym/10)), 5,  cv::Scalar(255, 0, 0), 5, 8, 0);
            }
        }

		else {
			ROS_INFO("Image empty");
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
}
