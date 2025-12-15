/**
 * 진행률 바 컴포넌트
 */
import './ProgressBar.css'

interface ProgressBarProps {
  progress: number // 0-100
  label?: string
}

export const ProgressBar = ({ progress, label }: ProgressBarProps) => {
  const clampedProgress = Math.min(100, Math.max(0, progress))

  return (
    <div className="progress-bar-wrapper">
      {label && <div className="progress-bar-label">{label}</div>}
      <div className="progress-bar">
        <div
          className="progress-bar-fill"
          style={{ width: `${clampedProgress}%` }}
        >
          <span className="progress-bar-text">{Math.round(clampedProgress)}%</span>
        </div>
      </div>
    </div>
  )
}
