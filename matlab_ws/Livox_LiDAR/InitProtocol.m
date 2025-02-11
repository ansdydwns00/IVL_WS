function InitProtocol(~,~,udpObj)
slaveIP = '192.168.1.175'; 
% slaveIP = '192.168.1.119'; 
slavePort = 65000;   

% 핸드셰이크 메시지 구성
handshakeMsg = uint8([0xaa,0x01,0x19,0x00,0x01,0x02,0x00,0xb0,0x31,0x00,0x01,0xc0,0xa8,0x01,0x32,0xc1,0xda,0xcd,0xd8,0xc1,0xda,0x28,0xc6,0xc5,0xdd]);

% 핸드셰이크 메시지 전송
write(udpObj, handshakeMsg, slaveIP, slavePort);
% pause(0.5)


% 메시지 구성
queryMsg = uint8([0xaa,0x01,0x0f,0x00,0x00,0x03,0x00,0x6c,0xfd,0x00,0x02,0xda,0xe4,0x56,0x36]);

% 메시지 전송
write(udpObj, queryMsg, slaveIP, slavePort);
% pause(0.5)


% 메시지 구성
query2Msg = uint8([0xaa,0x01,0x0f,0x00,0x00,0x04,0x00,0x64,0xb0,0x00,0x02,0x1e,0xe2,0xe5,0x96]);

% 메시지 전송
write(udpObj, query2Msg, slaveIP, slavePort);


% 메시지 구성
heartBeatMsg = uint8([0xaa,0x01,0x0f,0x00,0x00,0x05,0x00,0xbc,0xa9,0x00,0x03,0xa4,0x08,0xe9,0xda]);

% 메시지 전송
write(udpObj, heartBeatMsg, slaveIP, slavePort);


unknownMsg = uint8([0xaa,0x01,0x10,0x00,0x00,0x06,0x00,0x68,0x5d,0x00,0x04,0x01,0xb4,0x5f,0x19,0xa9]);

% 메시지 전송
write(udpObj, unknownMsg, slaveIP, slavePort);
end