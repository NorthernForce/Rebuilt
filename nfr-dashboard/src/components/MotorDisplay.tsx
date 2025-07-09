import { useEntry } from "@frc-web-components/react";

interface MotorDisplayProps {
    motorId: number;
}

function MotorDisplay({ motorId }: MotorDisplayProps) {
    const [type] = useEntry(`FWC/Motors/${motorId}/Type`, 'Unknown');
    switch (type) {
    case 'TalonFX':
        return (
            <TalonFXDisplay motorId={motorId} />
        );
    case 'TalonFXS':
        return (
            <TalonFXSDisplay motorId={motorId} />
        );
    case 'SparkMax':
        return (
            <SparkMaxDisplay motorId={motorId} />
        );
    }
}