import { Canvas, CanvasMjpgStream, NetworkAlerts, useEntry } from "@frc-web-components/react";
import "./Teleop.css"

function TimeDisplay(props: { time: number }) {
    let minutes = Math.floor(props.time / 60);
    let seconds = Math.floor(props.time % 60);
    let secondsString = seconds < 10 ? `0${seconds}` : `${seconds}`;
    return <div className="time-display">
        {minutes}:{secondsString}
    </div>
}

function Teleop() {
    let [time] = useEntry("/FWC/MatchTime", 0);
    let ip = new URLSearchParams(window.location.search).get("ip") || "10.1.72.2"
    let visionCameraIp = ip == "10.1.72.2" ? "10.1.72.36:1181" : "172.22.11.2:5807"
    return <>
        <div className="teleop-container">
            <NetworkAlerts source-key="/SmartDashboard/Alerts" />
            <div className="time-display">
                <TimeDisplay time={time} />
            </div>
            <Canvas className="camera-feed">
                <CanvasMjpgStream origin={[0,0]} crosshairColor="white" srcs={[`http://${visionCameraIp}/stream.mjpg`]} />
            </Canvas>
            <div></div>
        </div>
    </>
}

export default Teleop;