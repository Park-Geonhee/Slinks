package com.nemo.neplan.model;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;

import javax.persistence.*;
import java.io.Serializable;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

@Entity
@JsonIgnoreProperties({"hibernateLazyInitializer", "handler"})
public class Plan implements Serializable {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private long id;

    @Column(nullable=false)
    private String title;

    private String depDatetime;

    private boolean isPublic=false;

    // User과 관련된 필드 선언
    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "user_id")
    @JsonIgnoreProperties({"hibernateLazyInitializer", "handler", "createdDate", "modifiedDate"})
    private User user;

    // File과 관련된 필드 선언
    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "file_id")
    @JsonIgnoreProperties({"hibernateLazyInitializer", "handler"})
    private File file;

    @PrePersist
    public void setDepDatetimeOnCreate() {
        if (depDatetime == null) {
            // 현재 시각을 기준으로 하루를 더해줍니다.
            LocalDateTime tomorrow = LocalDateTime.now().plusDays(1);

            // 원하는 형식으로 변환합니다. 예: "201211251010"
            DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyyMMddHHmm");
            depDatetime = tomorrow.format(formatter);
        }
    }

    public long getId() {
        return id;
    }

    public void setId(long id) {
        this.id = id;
    }

    public String getTitle() {
        return title;
    }

    public void setTitle(String title) {
        this.title = title;
    }

    public String getDepDatetime() {
        return depDatetime;
    }

    public void setDepDatetime(String depDatetime) {
        this.depDatetime = depDatetime;
    }

    public boolean isPublic() {
        return isPublic;
    }

    public void setPublic(boolean aPublic) {
        isPublic = aPublic;
    }

    public User getUser() {
        return user;
    }

    public void setUser(User user) {
        this.user = user;
    }

    public File getFile() {
        return file;
    }

    public void setFile(File file) {
        this.file = file;
    }

    public int getMaxPlaceOrder() {
        EntityManager entityManager = Persistence.createEntityManagerFactory("your-persistence-unit-name").createEntityManager();

        // 해당 Plan에 속한 PlanPlace 엔티티 중 가장 큰 placeOrder 값을 조회합니다.
        Query query = entityManager.createQuery("SELECT MAX(pp.placeOrder) FROM PlanPlace pp WHERE pp.plan = :plan");
        query.setParameter("plan", this);

        Integer maxPlaceOrder = (Integer) query.getSingleResult();

        // maxPlaceOrder 값이 null인 경우(플레이리스트가 비어있는 경우) 0을 반환합니다.
        return maxPlaceOrder != null ? maxPlaceOrder : 0;
    }

}
