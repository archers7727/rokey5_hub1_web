/**
 * 진행률 표시 컴포넌트
 */
import './ProgressIndicator.css'

interface ProgressIndicatorProps {
  current: number
  total: number
}

export const ProgressIndicator = ({ current, total }: ProgressIndicatorProps) => {
  return (
    <div className="progress-indicator">
      <div className="progress-steps">
        {Array.from({ length: total }, (_, i) => i + 1).map((step) => (
          <div
            key={step}
            className={`progress-step ${step <= current ? 'active' : ''} ${
              step === current ? 'current' : ''
            }`}
          >
            <div className="progress-step-circle">{step}</div>
            {step < total && <div className="progress-step-line" />}
          </div>
        ))}
      </div>
      <div className="progress-text">
        {current} / {total}
      </div>
    </div>
  )
}
