import { Field, FieldPath, FieldRobot, useEntry } from "@frc-web-components/react";

interface Field2dProps {
    width?: number;
    sourceKey: string;
}

function Field2d({ width, sourceKey }: Field2dProps) {
    const [game] = useEntry(`${sourceKey}/game`, "Reefscape");
    const [robotPose] = useEntry(`${sourceKey}/robotPose`, [0, 0, 0]);
    const [robotPath] = useEntry(`${sourceKey}/robotPath`, []);
    const [alliance] = useEntry(`${sourceKey}/alliance`, "red");
    return (
        <div style={{ justifyItems: 'center', display: 'flex', width: '100%' }}>
            <Field style={{ width: width || '100%' }} game={game || "Reefscape"}>
                {robotPose && (
                    <FieldRobot
                        pose={robotPose}
                        unit="radians"
                        color={alliance}
                    />
                )}
                {robotPath && (
                    <FieldPath
                        poses={robotPath}
                    />
                )}
            </Field>
        </div>
    );
}

export default Field2d;