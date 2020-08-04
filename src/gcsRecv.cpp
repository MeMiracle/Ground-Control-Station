#include <ros/ros.h>
#include <uavmessage.pb.h>
#include <uavmessage.pb.cc>
#include <taskmessage.pb.h>
#include <taskmessage.pb.cc>
#include <feedbackdata.pb.h>
#include <feedbackdata.pb.cc>
#include <uwblpsmessage.pb.h>
#include <uwblpsmessage.pb.cc>
#include <string>
#include <fstream>
#include <pthread.h>
#include <zmq.h>

using namespace std;

#define pi 3.1415926

void *subscriber;

char uav_address_config[] = "gcs_test/configure/uav_address_config.txt";

char gps_log[] = "gcs_test/log/gps_log.txt";
char dist_log[] = "gcs_test/log/dist_log.txt";
char vel_log[] = "gcs_test/log/vel_log.txt";
char waypoint_log[] = "gcs_test/log/waypoint_log.txt";
char uwb_log[] = "gcs_test/log/uwb.txt";

int tgt_uav_id = 1;//默认接收数据的无人机

void *chooseAddrForUAV(void *arg)
{
    string temp_address((char*)arg);
    ROS_INFO("temp_address=%s",temp_address.c_str());

    while(ros::ok())
    {
        ROS_INFO("Input the tgt_uav_id to get msg:");
        
        scanf("%d",&tgt_uav_id);

        if(tgt_uav_id>0 && tgt_uav_id<256)
        {
            tgt_uav_id = 5000 + tgt_uav_id;

            char port[4];
        
            sprintf(port,"%d",tgt_uav_id);

            char new_address[128] = {'0'};
            int count = 0;
            for(int i=0;i<temp_address.size();i++)
            {
                new_address[i] = temp_address.data()[i];

                if(temp_address.data()[i] == ':')
                {
                    count++;

                    if(count>=2)
                    {
                        break;
                    }
                }               
            }
            
             sprintf(new_address,"%s%s%c",new_address,port,'\0');

             ROS_INFO("new_address = %s",new_address);

             zmq_close(subscriber);
        }

        else
        {
            ROS_INFO("error input.\n");
        }
        
    }
}

string getDirectory()
{
    char abs_path[1024];
    int cnt = readlink("/proc/self/exe", abs_path, 1024);//获取可执行程序的绝对路径
    if(cnt < 0|| cnt >= 1024)
    {
        return NULL;
    }

    //最后一个'/' 后面是可执行程序名，去掉devel/lib/m100/exe，只保留前面部分路径
    int count=0;
    for(int i = cnt; i >= 0; --i)
    {
        if(abs_path[i]=='/')
        {
            abs_path[i + 1]='\0';
            count++;

            if(count == 4)
            {
                break;
            }
        }
    }

    string path(abs_path);

    path = path + "src/";

    return path;
}


int main(int argc, char *argv[])
{
    /* code for main function */
    ros::init(argc, argv, "gcsRecv");
    ros::NodeHandle nh;

    ROS_INFO("gcsRecv_node starting.");

    void *context = zmq_init(1);

connect:
    subscriber = zmq_socket(context,ZMQ_SUB);

    string workspace_path = getDirectory();

    string config_file(workspace_path + uav_address_config);
    fstream ifs;
    ifs.open(config_file.c_str(),ios::in | ios::out);
    if(!ifs)
    {
        printf("%s ",config_file.c_str());
        perror("open error:");
        return -1;
    }

    string uav_address;
    if(getline(ifs,uav_address))
    {
        //pthread_t thread;
        //pthread_create(&thread,NULL,chooseAddrForUAV,(void*)uav_address.c_str());

        ROS_INFO("trying to connect to %s",uav_address.c_str());
        int count=0;
        while(ros::ok())
        {
            int ret = zmq_connect(subscriber,(char*)uav_address.c_str());
        
            if(ret != 0)
            {
                perror("connecting error:");

                count++;
                if(count>=3)
                {
                    ROS_INFO("Can not connect to %s !",uav_address.c_str());
                    return -1;
                }
            }

            else
            {
                ROS_INFO("connecting success.");
                break;
            }

            sleep(1);
        }
    }

    ifs.close();

    //char filter = 10;
    //int filter = 254;
    //ret = zmq_setsockopt(mv_subscriber,ZMQ_SUBSCRIBE,&filter,sizeof(filter));
    int ret = zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "", 0);

    if(ret != 0)
    {
        perror("set socket error:");
        return 0;
    }


    //持续接收数据
    ROS_INFO("receiving navmsg from uav_%d...",tgt_uav_id);
    while(ros::ok())
    {
        zmq_msg_t recv_msg;

        zmq_msg_init(&recv_msg);
        int ret = zmq_msg_recv(&recv_msg,subscriber,0);

        if(ret < 0)
        {
            perror("subscriber error:");
            //break;
            goto connect;
        }

        else if(ret == 0)
        {
            ROS_INFO("subscriber no message.");
        }

        else
        {
            ROS_INFO("subscriber zmq_msg_t %d bytes.",ret);
        }

/*      //zmq_msg_t消息转成char
        int size = zmq_msg_size(&recv_msg);
        char *ch = (char*)malloc(ret);

        memcpy(ch,zmq_msg_data(&recv_msg),ret);
        //str[size] = 0;

        for(int i=0;i<ret;i++)
        {
            ROS_INFO("%d ",ch[i]);
        }
        ROS_INFO("\n");
*/        
        string str;
        str.assign((char*)zmq_msg_data(&recv_msg),ret);
        //string str = static_cast<string>(ch);
        //string str.to_string(ch);// = (string)malloc(ret);
        //memcpy(&str,zmq_msg_data(&recv_msg),ret);
        zmq_msg_close(&recv_msg); 
        //free(ch);

/*        //打印string
        ROS_INFO("mv_subscriber stirng %d bytes:",(int)str.size());
        for(int i=0;i<ret;i++)
        {
            ROS_INFO("%d ",str[i]);
        }
        ROS_INFO("\n");
*/
        uavMessage::Message msg;
        msg.ParseFromString(str);
        
        if( msg.msghead().tgt_uav_id()[0] == 0)
        {
            string playload_str;
            playload_str = msg.playload();

            if(msg.msghead().msg_type() == 4)
            {
                //飞行状态反馈数据
                if(msg.mutable_msghead()->msg_id()==3 && msg.mutable_msghead()->topic_id()==5)
                {
                    feedbackData::FeedbackData fd_data;
                    fd_data.Clear();
                    fd_data.ParseFromString(msg.playload());

                    ROS_INFO("batteryPercent=%f,state=%d",fd_data.batterypercent(),fd_data.state());
                    ROS_INFO("dist position:(%lf,%lf,%lf)",fd_data.px(),fd_data.py(),fd_data.pz());
                    ROS_INFO("gps position:(%lf,%lf,%lf)",fd_data.lon(),fd_data.lat(),fd_data.alt());
                    ROS_INFO("roll=%lf,pitch=%lf,yaw=%lf",fd_data.roll(),fd_data.pitch(),fd_data.yaw());
                    ROS_INFO("vx=%lf,vy=%lf,vz=%lf,vw=%lf",fd_data.vx(),fd_data.vy(),fd_data.vz(),fd_data.vw());
                    ROS_INFO("ax=%lf,ay=%lf,az=%lf\n",fd_data.ax(),fd_data.ay(),fd_data.az());

                    // ofstream gps_str;
                    // string gps_file_name(getDirectory() + gps_log);
                    // gps_str.open(gps_file_name.c_str(),ios::out | ios::app);

                    // if(gps_str)
                    // {
                    //     gps_str << fd_data.lon() << " " << fd_data.lat() << " " << fd_data.alt() << endl;

                    //     gps_str.close();
                    // }
                    // else
                    // {
                    //     printf("%s ",gps_file_name.c_str());
                    //     perror("open failed: ");
                    // }

                    // ofstream dist_str;
                    // string dist_file_name(getDirectory() + dist_log);
                    // dist_str.open(dist_file_name.c_str(), ios::out | ios::app);

                    // if(dist_str)
                    // {
                    //     dist_str << fd_data.px() << " " << fd_data.py() << " " << fd_data.pz() << endl;
                        
                    //     dist_str.close();
                    // }
                    // else
                    // {
                    //     printf("%s ",dist_file_name.c_str());
                    //     perror("open failed: ");
                    // }

                    // ofstream vel_str;
                    // string vel_file_name(getDirectory() + vel_log);
                    // vel_str.open(vel_file_name.c_str(), ios::out | ios::app);

                    // if(vel_str)
                    // {
                    //     vel_str << fd_data.vx() << " " << fd_data.vy() << " " << fd_data.vz() << endl;

                    //     vel_str.close();
                    // }
                    // else
                    // {
                    //     printf("%s ",vel_file_name.c_str());
                    //     perror("open failed: ");
                    // }
                }    


                //航迹规划结果
                if(msg.msghead().msg_id()==7 && msg.msghead().topic_id()==9)
                {
                    // myTaskMessage::TaskMessage waypoints;
                    // waypoints.Clear();

                    // waypoints.ParseFromString(msg.playload());

                    // ofstream waypoints_str;
                    // string waypoints_file_name(getDirectory() + waypoint_log);

                    // waypoints_str.open(waypoints_file_name.c_str(),ios::out|ios::app);

                    // if(waypoints_str)
                    // {
                    //     for(int i=0;i<waypoints.point_size();i++)
                    //     {
                    //         waypoints_str << waypoints.point(i).x() << " " << waypoints.point(i).y() << " " << waypoints.point(i).z() << endl;
                    //     }

                    //     waypoints_str.close();
                    // }
                    // else
                    // {
                    //     printf("%s ",waypoints_file_name.c_str());
                    //     perror("open failed: ");
                    // }   
                }

                if(msg.msghead().msg_id()==10 && msg.msghead().topic_id()==12)
                {
                    // uwblpsMessage::uwblpsMessage uwb_pos;
                    // uwb_pos.Clear();

                    // uwb_pos.ParseFromString(msg.playload());

                    
                    // ofstream uwb_str;
                    // string uwb_file_name(getDirectory() + uwb_log);

                    // uwb_str.open(uwb_file_name.c_str(), ios::out|ios::app);

                    // if(uwb_str && msg.msghead().src_uav_id()==1)
                    // {
                    //     uwb_str << uwb_pos.linear_x() << " " << uwb_pos.linear_y() << " " << uwb_pos.linear_z() << endl;
                    //     uwb_str.close();
                    // }
                    // else
                    // {
                    //     printf("%s ",uwb_file_name.c_str());
                    //     perror("open failed: ");
                    // }   
                }
            }
        }
    }

    zmq_close(subscriber);
    zmq_term(context);

    ros::shutdown();
    return 0;
}