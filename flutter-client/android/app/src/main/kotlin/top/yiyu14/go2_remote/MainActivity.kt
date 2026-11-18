package top.yiyu14.go2_remote

import android.hardware.input.InputManager
import android.os.Handler
import android.view.KeyEvent
import android.view.MotionEvent
import io.flutter.embedding.android.FlutterActivity
import org.flame_engine.gamepads_android.GamepadsCompatibleActivity

// 手柄插件要求宿主 Activity 把 KeyEvent / MotionEvent 转发给它，并且注册
// 输入设备监听 —— 系统不会主动告诉 Flutter「插了一个手柄」。这一段是
// `gamepads` 包文档里的标准样板，少了它安卓上的手柄完全收不到事件。
class MainActivity : FlutterActivity(), GamepadsCompatibleActivity {
    var keyListener: ((KeyEvent) -> Boolean)? = null
    var motionListener: ((MotionEvent) -> Boolean)? = null

    override fun dispatchGenericMotionEvent(motionEvent: MotionEvent): Boolean {
        if (motionListener?.invoke(motionEvent) == true) {
            return true
        }
        return super.dispatchGenericMotionEvent(motionEvent)
    }

    override fun dispatchKeyEvent(keyEvent: KeyEvent): Boolean {
        if (keyListener?.invoke(keyEvent) == true) {
            return true
        }
        return super.dispatchKeyEvent(keyEvent)
    }

    override fun registerInputDeviceListener(
        listener: InputManager.InputDeviceListener,
        handler: Handler?,
    ) {
        val inputManager = getSystemService(INPUT_SERVICE) as InputManager
        inputManager.registerInputDeviceListener(listener, null)
    }

    override fun registerKeyEventHandler(handler: (KeyEvent) -> Boolean) {
        keyListener = handler
    }

    override fun registerMotionEventHandler(handler: (MotionEvent) -> Boolean) {
        motionListener = handler
    }
}
