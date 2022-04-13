#include "labview_interface.h"
using namespace std;



LabViewInterface::LabViewInterface(PCSTR IP, int PORT)
	:vf(vector<float>(7))
{
	std::cout << "labview_interface: constructing" << std::endl;

	//winsock2の初期化
	WSAStartup(MAKEWORD(2, 0), &wsaData);
	//ソケットの作成
	sock = socket(AF_INET, SOCK_STREAM, 0);

	//接続先指定用構造体の準備
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(PORT);
	//server.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	InetPton(AF_INET, IP, &server_addr.sin_addr.s_addr);


	//サーバに接続
	connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr));

	memset(buf, 0, MAX_BUF_SIZE);
	send(sock, "Eabc1;", MAX_BUF_SIZE, 0);
	recv(sock, buf, MAX_BUF_SIZE, 0);
	std::cout << "first buf " << buf << std::endl;

	constructed = true;
	std::cout << "labview_interface: constructed" << std::endl;

}


void LabViewInterface::update() {

	if (!constructed) return;

	//if(CNT) std::cin >> buf;
	//サーバからデータを受信
	send(sock, "Eabc1;", MAX_BUF_SIZE, 0);
	recv(sock, buf, MAX_BUF_SIZE, 0);

	int cnt = 0;
	std::string tmp;
	for (int i = 1; i < sizeof(buf); i++) {
		if (buf[i] == ',') {
			vf[cnt] = stof(tmp);
			tmp = "";
			cnt++;
		}
		else {
			tmp += buf[i];
		}
		if (cnt >= 6) break;
	}

	//std::cout << "labview_interface: ";
	//std::cout <<"buf"<<buf << std::endl;
	//for (int i = 0; i < 7; i++) std::cout << vf[i] << " ";
	//std::cout << std::endl;
	cnt_++;

	//基本姿勢時
	//0.055 0.002 0.036 - 90.001 8 90.002 0
	//横向き時
	//6.505 -1.794 -4.326 -111.032 9.508 106.995 0

	for (int i = 0; i <= 2; i++) vf[i] /= 1000.;
	vf[2] = -vf[2];
	vf[0] += -0.14725;
	vf[1] += -0.30875;
	vf[2] += 0.23502;

	for (int i = 3; i <= 6; i++) vf[i] *= M_PI / 180.;


	//float T = vf[3], A = vf[4], O = vf[5];
	//std::vector<float> q(4);
	//q[0] = cos(O / 2) * sin(A / 2) * sin(T / 2) - sin(O / 2) * sin(A / 2) * cos(T / 2);
	//q[1] = cos(O / 2) * sin(A / 2) * cos(T / 2) + sin(O / 2) * sin(A / 2) * sin(T / 2);
	//q[2] = sin(O / 2) * cos(A / 2) * cos(T / 2) + cos(O / 2) * cos(A / 2) * sin(T / 2);
	//q[3] = cos(O / 2) * cos(A / 2) * cos(T / 2) - sin(O / 2) * cos(A / 2) * sin(T / 2);

	//for (int i = 0; i < 4; i++) vf[3 + i] = q[i];





}


LabViewInterface::~LabViewInterface() {
	closesocket(sock);

	//winsock2の終了
	WSACleanup();
}

