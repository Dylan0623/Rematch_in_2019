wifi.setmode(wifi.SOFTAP,false)
config = {}
config.ssid = "Project_PHJ"
config.pwd = "12345678"
config.auth = wifi.WPA_WPA2_PSK
config.channel = 8
config.hidden = false
wifi.ap.config(config)

dhcp_config ={}
dhcp_config.start = "192.168.1.100"
wifi.ap.dhcp.config(dhcp_config)
wifi.ap.dhcp.start()

ip =0

wifi.eventmon.register(wifi.eventmon.AP_STACONNECTED,function(T)
    ip = wifi.ap.getip()
    print(ip)
    end
)

TRIG_PIN = 1
ECHO_PIN = 2
gpio.mode(TRIG_PIN, gpio.OUTPUT)
gpio.mode(ECHO_PIN, gpio.INT)
gpio.trig(ECHO_PIN, "both", echo_callback)

 

time_start = 0
time_stop = 0
distance = 0
distance_pre = 0
speed = 0
current = 0

scl = 3
sda = 4
i2c.setup(0, sda, scl, i2c.SLOW)
disp = u8g2.ssd1306_i2c_128x64_noname(0, 0x3c)
function print_oled()
    disp:firstPage()
    disp:drawStr(0,0,"速度："+speed)
    disp:drawStr(0,15,"位置："+distance)
    disp:drawStr(0,30,"电流："+current)
end

function trigger()
    gpio.write(TRIG_PIN, gpio.HIGH)
    tmr.delay(12)
    gpio.write(TRIG_PIN, gpio.LOW)
    --current = adc.read(0)
end

function calculate()
    local echo_time = (time_stop - time_start) / 1000000
    if echo_time > 0 then
        distance = echo_time * 340 
        speed = (distance - distance_pre)/0.1
    end
    else 
        speed = -1
    end
end

function echo_callback(level)
    if level == 1 then
        -- rising edge
        time_start = tmr.now()
    else
        -- falling edge
        time_stop = tmr.now()
        calculate()
    end
    print_oled()
end

mytimer = tmr.create()
mytimer:register(100, tmr.ALARM_AUTO, trigger)
mytimer:start()

uart.setup(0, 115200, 8, uart.PARITY_NONE, uart.STOPBITS_1, 1)
uart.on("data", "/n",
    function(data)
    current = data
    end, 0)

server = net.createServer(300)
server:listen(80,function(conn)  
  conn:on("receive",function(conn,payload)  
        local html = string.format("HTTP/1.0 200 OK\r\n"  
        .."Content-Type: text/html\r\n"  
        .."Connection: Close\r\n\r\n"  
        .."<title>信息</title>"
        .."<h1>当前信息</h1>"
        .."<h2>当前位置(m):"..distance.."</h2>"
        .."<h2>当前速度(m/s):"..speed.."</h2>"
        .."<h2>当前电流(mA):"..current.."</h2>"
        )
    conn:send(html)  
  end)    
end)

