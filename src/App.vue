<script setup></script>

<template>
  <div class="w-screen flex flex-col" style="overflow: auto; height: 100dvh">
    <div class="flex-1">
      <div class="center flex flex-col">
        <div class="h-36 flex flex-row mb-12">
          <div class="flex-1 lh-36">
            <span class="font-14-500 c-666">状态:&nbsp;&nbsp;</span>
            <span class="font-14-500" :style="{ color: wsStateMapper[wsState].color }">
              {{ wsStateMapper[wsState].txt }}
            </span>
          </div>

          <div class="flex-1">
            <span class="font-14-500 c-666">档位:&nbsp;&nbsp;</span>
            <span class="font-14-500" :style="{ color: gearMapper[gear].color }">
              {{ gearMapper[gear].txt }}
            </span>
          </div>

          <div style="flex: 2">
            <span class="font-14-500 c-666">最近命令:&nbsp;&nbsp;</span>
            <span class="font-14-500" :style="{ color: '#07c160' }">
              {{ currentCmd }}
            </span>
          </div>

          <div class="flex-1">
            <span class="font-14-500 c-666">时长:&nbsp;&nbsp;</span>
            <span class="font-14-500" :style="{ color: durationEn ? '#07c160' : '#969799' }">
              {{ formatterSec(duration) }}
            </span>
          </div>
        </div>
        <div class="flex-1 flex flex-row mb-12 justify-between">
          <div class="flex-1 py-4">
            <div class="flex flex-row items-center mb-12">
              <p class="font-14-500 c-666 mr-12 labelW">前进恒定速度开关:</p>
              <van-switch v-model="config.keepForwardSpeed" />
            </div>
            <div class="flex flex-row items-center mb-12">
              <p class="font-14-500 c-666 mr-12 labelW">前进时恒定速度:</p>
              <van-stepper disable-input :min="0" :max="255" v-model="config.forwardSpeed" step="5" />
            </div>

            <div class="flex flex-row items-center mb-12">
              <p class="font-14-500 c-666 mr-12 labelW">后退恒定速度开关:</p>
              <van-switch v-model="config.keepBackSpeed" />
            </div>
            <div class="flex flex-row items-center mb-12">
              <p class="font-14-500 c-666 mr-12 labelW">后退时恒定速度:</p>
              <van-stepper disable-input :min="0" :max="255" v-model="config.backSpeed" step="5" />
            </div>
          </div>
          <div class="road" style="visibility: hidden">
            <div class="car">
              <div class="wheel" style="top: 0; left: 0" :style="{ transform: `rotate(${wheelDeg}deg)` }"></div>
              <div class="line" style="top: 15px; left: 12px; right: 12px"></div>
              <div class="wheel" style="top: 0; right: 0" :style="{ transform: `rotate(${wheelDeg}deg)` }"></div>

              <div class="line w-2" style="top: 15px; left: 50%; bottom: 15px"></div>

              <div class="wheel" style="bottom: 0; left: 0"></div>
              <div class="line" style="bottom: 15px; left: 10px; right: 10px"></div>
              <div class="wheel" style="bottom: 0; right: 0"></div>
            </div>
            <div class="scroll-element" id="primary"></div>
            <div class="scroll-element" id="secondary"></div>
          </div>
          <div class="flex-1 py-32 pl-24">
            <div class="flex flex-row items-center mb-12">
              <p class="font-14-500 c-666 mr-12 labelW">转向速度:</p>
              <van-stepper disable-input :min="30" :max="255" v-model="config.trunSpeed" @change="trunSpeedChange"
                step="1" />
            </div>
          </div>
        </div>
        <div class="flex flex-row justify-between items-center" style="height: max-content">
          <div class="flex flex-row justify-center">
            <Btn :default-pos="offset" :acceleration="config.forwardAcceleration" @onChange="runningChange"
              :min="config.forwardMinSpeed" :max="config.forwardMaxSpeed" :step="1">
              <span class="font-16-500 c-333">前进</span>
            </Btn>
            <div class="w-20"></div>
            <Btn :default-pos="offset" :acceleration="config.backwardAcceleration" @onChange="backChange"
              :min="config.backwardMinSpeed" :max="config.backwardMaxSpeed" :step="1">
              <span class="font-16-500 c-333">倒车</span>
            </Btn>
          </div>

          <div class="flex flex-row justify-center">
            <van-button icon="setting-o" plain type="info" class="w-90" @click="showConfig = true">
              <span>设置</span>
            </van-button>
            <div class="w-20"></div>
            <van-button icon="pause-circle-o" :disabled="wsState != 2 || state == 2" plain type="danger" class="w-90"
              @click="stop">
              <span>停止</span>
            </van-button>
            <div class="w-20"></div>
            <van-button icon="replay" :disabled="wsState != 2 || state == 1" plain type="primary" class="w-90"
              @click="start">
              <span>启动</span>
            </van-button>
          </div>

          <div class="flex flex-row justify-center">
            <Btn :default-pos="0" @onChange="anglesChange" :step="1" :min="-1" :max="1">
              <span class="font-16-500 c-333">左</span>
            </Btn>
            <div class="w-20"></div>
            <Btn :default-pos="0" @onChange="anglesChange" :step="-1" :min="-1" :max="1">
              <span class="font-16-500 c-333">右</span>
            </Btn>
          </div>
        </div>
      </div>
    </div>

    <van-popup v-model:show="showConfig" position="left"
      :style="{ width: '70dvw', height: '100dvh', overflow: 'hidden' }">
      <div class="config-box flex flex-row px-24 py-12">
        <div class="flex-1">
          <div class="flex flex-row items-center mb-12">
            <p class="font-14-500 c-666 mr-12 labelW">前进速度区间:</p>
            <van-stepper disable-input :min="100" :max="255" v-model="config.forwardMinSpeed" class="mr-8" step="1" />
            <van-stepper disable-input :min="100" :max="255" v-model="config.forwardMaxSpeed" step="1" />
          </div>
          <div class="flex flex-row items-center mb-12">
            <p class="font-14-500 c-666 mr-12 labelW">后退速度区间:</p>
            <van-stepper disable-input :min="100" :max="255" v-model="config.backwardMinSpeed" class="mr-8" step="1" />
            <van-stepper disable-input :min="100" :max="255" v-model="config.backwardMaxSpeed" step="1" />
          </div>
          <div class="flex flex-row items-center mb-12">
            <p class="font-14-500 c-666 mr-12 labelW">前进加速度:</p>
            <van-stepper disable-input :min="0" :max="15" v-model="config.forwardAcceleration" step="1" />
          </div>
          <div class="flex flex-row items-center mb-12">
            <p class="font-14-500 c-666 mr-12 labelW">后退加速度:</p>
            <van-stepper disable-input :min="0" :max="15" v-model="config.backwardAcceleration" step="1" />
          </div>
        </div>
        <div class="flex-1">
          <div class="flex flex-row items-center mb-12">
            <van-button icon="revoke" plain type="warning" class="w-90" @click="resetDefault">
              <span>恢复默认值</span>
            </van-button>
          </div>
          <div class="flex flex-row items-center mb-12">
            <van-field v-model="ip" @input="ipChange" label="IP" placeholder="请输入IP" />
          </div>
        </div>
      </div>
    </van-popup>
  </div>
</template>

<script>
import Slider from './components/Slider.vue';
import Btn from './components/Btn.vue';
import { formatterSec } from './config';
export default {
  components: { Slider, Btn },
  data() {
    return {
      rotate: 0,
      formatterSec: formatterSec,
      duration: 0,
      showConfig: false,
      durationEn: false,
      ip: '192.168.208.169',
      lastCmd: '',
      checked: false,
      angles: 0,
      lastAngles: 0,
      config: {
        trunRange: 60,
        forwardSpeed: 10,
        keepForwardSpeed: false,
        keepBackSpeed: false,
        backSpeed: 10,
        trunSpeed: 1,
        forwardAcceleration: 5,
        backwardAcceleration: 5,
        motorEscMiddle: 75,
        motorEscForWardMin: 76,
        motorEscForWardMax: 99,
        motorEscBackWardMin: 25,
        motorEscBackWardMax: 73,
        forwardMinSpeed: 100,
        forwardMaxSpeed: 255,
        backwardMinSpeed: 100,
        backwardMaxSpeed: 255,
      },
      currentCmd: '',
      preAngles: -1,
      wheelDeg: 0,
      running: 0,
      preRunning: -1,
      offset: 0,
      gear: 'FORWARD', // FORWARD 前进 BACKWARD 后退 3 停止
      state: 1, //1 启动 2 停止
      wsState: 1, //1 连接中 2 已连接 3 断开连接
      gearMapper: {
        FORWARD: {
          txt: '前进',
          color: '#07c160',
        },
        BACKWARD: {
          txt: '后退',
          color: '#FAAB0C',
        },
        STOP: {
          txt: '停止',
          color: '#969799',
        },
      },
      wsStateMapper: {
        1: {
          txt: '连接中',
          color: '#969799',
        },
        2: {
          txt: '已连接',
          color: '#07c160',
        },
        3: {
          txt: '断开连接',
          color: '#ee0a24',
        },
      },
      setup: false,
      fixedSpeed: false,
    };
  },
  computed: {
    turnMax() {
      return this.config.trunRange / 2 + 90;
    },
    turnMin() {
      return 90 - this.config.trunRange / 2;
    },
  },
  created() {
    let configStore = localStorage.getItem('CONFIG') ? JSON.parse(localStorage.getItem('CONFIG')) : null;
    if (configStore) {
      Object.assign(this.config, configStore);
    }
    //ceshi
    this.initWebSocket();

    setInterval(() => {
      localStorage.setItem('CONFIG', JSON.stringify(this.config));
    }, 2000);
  },
  mounted() {
    setTimeout(() => {
      window.requestAnimationFrame(this.animation);
    }, 1500);
    setInterval(() => {
      if (this.durationEn) {
        this.duration += 1;
      }
    }, 1000);
  },
  methods: {
    resetDefault() {
      this.config = {
        trunRange: 60,
        forwardSpeed: 10,
        keepForwardSpeed: false,
        keepBackSpeed: false,
        backSpeed: 10,
        trunSpeed: 30,
        forwardAcceleration: 1,
        backwardAcceleration: 0,
        motorEscMiddle: 75,
        motorEscForWardMin: 76,
        motorEscForWardMax: 99,
        motorEscBackWardMin: 25,
        motorEscBackWardMax: 73,
        forwardMinSpeed: 0,
        forwardMaxSpeed: 255,
        backwardMinSpeed: 0,
        backwardMaxSpeed: 255,
      };
      this.onOpen();
    },
    ipChange() {
      this.initWebSocket();
    },
    anglesChange(data) {
      this.angles = data.val;
    },
    runningChange(data) {
      let val = data.val;
      let flag = data.flag;

      this.gear = 'FORWARD';
      if (!flag && val) {
        if (this.config.keepForwardSpeed) {
          this.running = this.config.forwardSpeed;
        } else {
          this.running = val;
        }
      }
      if (flag) {
        this.running = 0;
      }
    },
    backChange(data) {
      let val = data.val;
      let flag = data.flag;
      this.gear = 'BACKWARD';
      if (!flag && val) {
        if (this.config.keepBackSpeed) {
          this.running = this.config.backSpeed;
        } else {
          this.running = val;
        }
      }
      if (flag) {
        this.running = 0;
      }
    },
    runningSend(speed) {
      if (speed) {
        if (this.angles === 0) {
          this.sendMsg(this.gear, speed);
        }
      } else {
        if (!this.angles) {
          this.sendMsg('STOP', 0);
        }
      }
    },
    anglesSend(val) {
      if (!val) {
        return;
      }
      this.sendMsg('TURN', val);
    },
    animation() {
      if (this.wsState === 2) {
        this.anglesSend(this.angles);
        this.runningSend(this.running);
      }
      window.requestAnimationFrame(this.animation);
    },
    start() {
      this.state = 1;
      this.sendMsg('ON');
    },
    stop() {
      this.state = 2;
      this.reset();
      this.sendMsg('OFF');
    },
    trunSpeedChange() {
      this.sendMsg('SET_TURN', this.config.trunSpeed);
    },
    toggleGear() {
      this.gear = this.gear === 'FORWARD' ? 'BACKWARD' : 'FORWARD';
    },
    toggleFixedSpeed() {
      this.fixedSpeed = !this.fixedSpeed;
    },
    initWebSocket() {
      const gateway = ` ws://${this.ip}/ws`;
      this.websocket = new WebSocket(gateway);
      this.websocket.onopen = this.onOpen;
      this.websocket.onclose = this.onClose;
      this.websocket.onmessage = this.onMessage;
    },
    onOpen() {
      this.trunSpeedChange();
      this.durationEn = true;
      this.wsState = 2;
      this.start();
    },
    map_range(value, low1, high1, low2, high2) {
      return low2 + ((high2 - low2) * (value - low1)) / (high1 - low1);
    },
    sendMsg(cmd, val = '') {
      if (!cmd) {
        console.log('cmd is emtry');
        return;
      }
      let data = {
        COMMOND: cmd,
        VALUE: val,
      };
      if (this.lastCmd === 'STOP' && cmd === 'STOP') {
        return
      }
      this.lastCmd = cmd
      this.currentCmd = `${cmd}: ${val || '-'}`;
      try {
        this.websocket.send(JSON.stringify(data));
      } catch (error) { }
    },
    reset() {
      this.running = 0;
      this.angles = 0;
      this.fixedSpeed = false;
    },
    onClose() {
      this.durationEn = false;
      this.wsState = 3;
      this.reset();
      setTimeout(() => {
        this.initWebSocket();
      }, 1000);
    },
    onMessage() {
      console.log('onMessage');
    },
  },
};
</script>

<style scoped>
div {
  line-height: 18px;
}

.w-center-box {
  border: 1px solid;
  width: 100dvh;
}

/* .center {
  width: 100dvh;
  border: 1px solid;
  padding: 32px;
  height: calc(100dvw);
  transform-origin: center;
  transform: rotate(90deg) translateX(218px) translateY(218px);
} */

.center {
  width: 100dvh;
  position: absolute;
  padding: 12px;
  height: 100dvw;
  transform-origin: 50dvw 50dvw;
  transform: translateX(0dvw) rotate(90deg);
  /* transform: rotate(0deg) translateX(50%) translateY(-50%); */
}

.config-box {
  width: 100dvh;
  height: 70dvw;
  transform-origin: 35dvw 35dvw;
  transform: translateX(0dvw) rotate(90deg);
}

.label {
  writing-mode: vertical-rl;
  text-orientation: upright;
  transform: rotate(90deg);
  float: right;
}

.rotate-90 {
  transform: rotate(90deg);
}

.rotate-90-center {
  transform: rotate(90deg) translateX(50%);
}

.rotate-180 {
  transform: rotate(270deg);
}

.h-200 {
  height: 200px;
}

.w-200 {
  width: 200px;
}

.hold-box {
  position: absolute;
  width: 25px;
  height: 25px;
  top: 50%;
  left: 50%;
  transform: translateX(-50%) translateY(-50%);
}

.road {
  position: relative;
  border: 1px solid;
  width: 120px;
  height: 100%;
  display: flex;
  background-color: #f0f0f0;
  justify-content: center;
  overflow: hidden;
}

.car {
  width: 60%;
  height: 70%;
  margin-top: 30px;
  /* border: 1px solid; */
  position: relative;
  z-index: 2;
}

.wheel {
  position: absolute;
  border: 2px solid;
  width: 10px;
  border-radius: 50px;
  height: 30px;
  transition: all 0.1;
}

.line {
  position: absolute;
  border: 1px solid;
  border-radius: 20px;
}

.scroll-element {
  z-index: 1;
  width: 12px;
  height: 100%;
  position: absolute;
  left: 50%;
  /* top: 0%; */
  transform: translateX(-50%);
  background: linear-gradient(to bottom,
      #fff 8.33%,
      #e0e0e0 8.33% 24.99%,
      #fff 24.99% 41.66%,
      #e0e0e0 41.66% 58.32%,
      #fff 58.32% 74.98%,
      #e0e0e0 74.98% 91.64%,
      #fff 91.64%);
}

.labelW {
  width: 120px;
}

.scroll-element#primary {
  animation: primary 0s linear infinite;
}

.scroll-element#secondary {
  animation: secondary 0s linear infinite;
}

@keyframes primary {
  from {
    top: 0%;
  }

  to {
    top: 100%;
  }
}

@keyframes secondary {
  from {
    top: -100%;
  }

  to {
    top: 0%;
  }
}
</style>
