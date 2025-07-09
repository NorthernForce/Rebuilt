import { NetworkAlerts, useEntry } from "@frc-web-components/react";
import AutoSelector from "../components/AutoSelector";

import './Auto.css';
import { fieldConfigs } from "@frc-web-components/fwc";
import Field2d from "../components/Field2d";

fieldConfigs.push(
    {
      game: 'Reefscape',
      image: './field-images/2025-field.jpg',
      corners: {
        topLeft: [425, 88],
        bottomRight: [3348, 1445],
      },
      size: [57.5721785, 26.417323],
      unit: 'foot',
    }
)

function Auto() {
    const [choreoAlertsWarning] = useEntry('/SmartDashboard/Choreo Alerts/warnings', []);
    const [choreoAlertsError] = useEntry('/SmartDashboard/Choreo Alerts/errors', ["ERRORS NOT PUBLISHED"]);
    const [choreoAlertsInfo] = useEntry('/SmartDashboard/Choreo Alerts/infos', []);
    const [alertsWarning] = useEntry('/SmartDashboard/Alerts/warnings', []);
    const [alertsError] = useEntry('/SmartDashboard/Alerts/errors', ["ERRORS NOT PUBLISHED"]);
    const [alertsInfo] = useEntry('/SmartDashboard/Alerts/infos', []);
    const [matchNumber] = useEntry('/FMSInfo/MatchNumber', 0);
    const combinedInfo = [...choreoAlertsInfo, ...alertsInfo];
    const combinedWarning = [...choreoAlertsWarning, ...alertsWarning];
    const combinedError = [...choreoAlertsError, ...alertsError];

    if (matchNumber == 0) {
        combinedError.push("Check alliance color; FMS info may be incorrect");
    }

    return (
        <>
            <div className="auto-container">
                <div className="auto-selector-container">
                    <AutoSelector sourceKey="/SmartDashboard/AutoChooser"/>
                </div>
                <div className="auto-field-container">
                    <Field2d width={800} sourceKey="/SmartDashboard/Field2d" />
                    <NetworkAlerts className="auto-alerts" infos={combinedInfo} warnings={combinedWarning}
                        errors={combinedError} />
                </div>
            </div>
        </>
    );
}

export default Auto;