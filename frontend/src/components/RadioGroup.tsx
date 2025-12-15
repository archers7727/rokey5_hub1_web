/**
 * 라디오 그룹 컴포넌트
 */
import './RadioGroup.css'

interface RadioOption {
  value: string
  label: string
  sublabel?: string
}

interface RadioGroupProps {
  value: string
  onChange: (value: string) => void
  options: RadioOption[]
  name: string
}

export const RadioGroup = ({ value, onChange, options, name }: RadioGroupProps) => {
  return (
    <div className="radio-group">
      {options.map((option) => (
        <label
          key={option.value}
          className={`radio-option ${value === option.value ? 'selected' : ''}`}
        >
          <input
            type="radio"
            name={name}
            value={option.value}
            checked={value === option.value}
            onChange={() => onChange(option.value)}
          />
          <span className="radio-label">
            {option.label}
            {option.sublabel && <span className="radio-sublabel">{option.sublabel}</span>}
          </span>
        </label>
      ))}
    </div>
  )
}
