import { createApp } from 'vue'
import './style.css'
import './common.css'
import 'amfe-flexible'
import App from './App.vue'

const app = createApp(App)
import 'vant/lib/index.css';
// vant use 
import { Button, Slider, Switch, Grid, GridItem, Icon, Stepper, Popup, Field } from 'vant';
app.use(Button)
app.use(Slider)
app.use(Switch)
app.use(Grid)
app.use(GridItem)
app.use(Icon)
app.use(Stepper)
app.use(Popup)
app.use(Field)

app.mount('#app')
