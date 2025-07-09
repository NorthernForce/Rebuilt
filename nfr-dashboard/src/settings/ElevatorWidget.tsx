import { RobotCommand, useEntry } from '@frc-web-components/react';
import './ElevatorWidget.css';

function ElevatorWidget(props: { name: string }) {
    const [position] = useEntry(`/FWC/${props.name}/Position`, 0);
    const [_, setTargetPosition] = useEntry(`/FWC/${props.name}/TargetPosition`, 0)
    return (
        <div className="elevator-widget">
            <h3>Position: {position}</h3>
            <h3>Target Position: <input onChange={e => setTargetPosition(Number(e.target.value))} /></h3>
            <RobotCommand className="elevator-target-position" name="Go to Target Position"
                source-key="/FWC/Elevator/TargetPosition" />
        </div>
    );
}
export default ElevatorWidget;