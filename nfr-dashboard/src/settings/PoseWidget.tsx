import { Field, FieldRobot, useEntry } from "@frc-web-components/react";
import './PoseWidget.css';

function PoseWidget() {
    const [pose] = useEntry('/FWC/Pose', [0, 0, 0]);
    const [robotWidth] = useEntry('/FWC/RobotWidth', 0.9652);
    const [robotLength] = useEntry('/FWC/RobotLength', 0.9652);
    return (
        <div className="pose-widget">
            <Field game="Reefscape" className="pose-field">
                <FieldRobot pose={pose} rotationUnit="radians" unit="meters" width={robotWidth} length={robotLength} />
            </Field>
        </div>
    );
}

export default PoseWidget;