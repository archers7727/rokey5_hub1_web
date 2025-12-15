/**
 * 선택 드롭다운 컴포넌트
 */
import './Select.css'

interface SelectProps {
  value: number | string
  onChange: (value: number) => void
  options: (number | string)[]
  label?: string
}

export const Select = ({ value, onChange, options, label }: SelectProps) => {
  return (
    <div className="select-wrapper">
      {label && <label className="select-label">{label}</label>}
      <select
        className="select"
        value={value}
        onChange={(e) => onChange(Number(e.target.value))}
      >
        {options.map((option) => (
          <option key={option} value={option}>
            {option}
          </option>
        ))}
      </select>
    </div>
  )
}
