/**
 * 숫자 입력 컴포넌트
 */
import './NumberInput.css'

interface NumberInputProps {
  value: number
  onChange: (value: number) => void
  min?: number
  max?: number
  label?: string
}

export const NumberInput = ({ value, onChange, min = 1, max = 10, label }: NumberInputProps) => {
  const handleDecrement = () => {
    if (value > min) {
      onChange(value - 1)
    }
  }

  const handleIncrement = () => {
    if (value < max) {
      onChange(value + 1)
    }
  }

  return (
    <div className="number-input-wrapper">
      {label && <label className="number-input-label">{label}</label>}
      <div className="number-input">
        <button
          className="number-input-btn"
          onClick={handleDecrement}
          disabled={value <= min}
        >
          −
        </button>
        <span className="number-input-value">{value}</span>
        <button
          className="number-input-btn"
          onClick={handleIncrement}
          disabled={value >= max}
        >
          +
        </button>
      </div>
    </div>
  )
}
